#include <iostream>
#include <cmath>
#include <string>
#include <stdexcept>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

// ---------------- Control table (XH540-W150-T, Protocol 2.0) ----------------
#define ADDR_OPERATING_MODE     11
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132

#define PROTOCOL_VERSION        2.0

// ---------------- Default settings ----------------
#define DEVICENAME              "/dev/ttyUSB0"
#define BAUDRATE                1000000

// X-series: 4096 ticks / rev
#define TICKS_PER_REV           4096.0

// Extended Position Control Mode
#define OPERATING_MODE_EXT_POS  4

#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0

// Extended position range is commonly ±1048575 ticks (check your config)
#define DEFAULT_MIN_GOAL_TICK   -1048575
#define DEFAULT_MAX_GOAL_TICK    1048575

// ---------------- Globals ----------------
dynamixel::PortHandler *portHandler = nullptr;
dynamixel::PacketHandler *packetHandler = nullptr;

static int g_dxl_id = 1;

static bool g_zero_on_start = true;
static bool g_have_angle0 = false;
static double g_angle0_deg = 0.0;

static double g_angle_scale = 20.0;         // e.g., gear ratio / mapping gain
static int32_t g_base_tick = 0;             // motor present position at start
static int32_t g_goal_offset_ticks = 0;     // extra offset ticks

static bool g_use_limits = true;
static int32_t g_min_goal_tick = DEFAULT_MIN_GOAL_TICK;
static int32_t g_max_goal_tick = DEFAULT_MAX_GOAL_TICK;

static double g_min_cmd_period = 0.01;      // seconds
static ros::Time g_last_cmd_time(0);

// ---- left/right data state ----
static bool g_have_left = false;
static bool g_have_right = false;
static double g_left_angle_deg = 0.0;
static double g_right_angle_deg = 0.0;

// ---------------- Helpers ----------------
static inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  return std::max(lo, std::min(hi, v));
}

// ---------------- Dynamixel functions ----------------
static bool write1B(int id, int addr, uint8_t val) {
  uint8_t dxl_error = 0;
  int comm = packetHandler->write1ByteTxRx(portHandler, id, addr, val, &dxl_error);
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("DXL write1B failed (id=%d addr=%d): %s", id, addr, packetHandler->getTxRxResult(comm));
    return false;
  }
  if (dxl_error != 0) {
    ROS_ERROR("DXL write1B error (id=%d addr=%d): %s", id, addr, packetHandler->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

static bool write4B(int id, int addr, uint32_t val) {
  uint8_t dxl_error = 0;
  int comm = packetHandler->write4ByteTxRx(portHandler, id, addr, val, &dxl_error);
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("DXL write4B failed (id=%d addr=%d): %s", id, addr, packetHandler->getTxRxResult(comm));
    return false;
  }
  if (dxl_error != 0) {
    ROS_ERROR("DXL write4B error (id=%d addr=%d): %s", id, addr, packetHandler->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

static bool read4B(int id, int addr, uint32_t &out) {
  uint8_t dxl_error = 0;
  int comm = packetHandler->read4ByteTxRx(portHandler, id, addr, &out, &dxl_error);
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("DXL read4B failed (id=%d addr=%d): %s", id, addr, packetHandler->getTxRxResult(comm));
    return false;
  }
  if (dxl_error != 0) {
    ROS_ERROR("DXL read4B error (id=%d addr=%d): %s", id, addr, packetHandler->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

static void disableTorque(int id) {
  write1B(id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

static void enableTorque(int id) {
  write1B(id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
}

static void setOperatingMode(int id, uint8_t mode) {
  // Must be torque OFF to change mode
  disableTorque(id);
  if (write1B(id, ADDR_OPERATING_MODE, mode)) {
    ROS_INFO("DXL id=%d Operating Mode set to %d (4=Extended Position)", id, (int)mode);
  }
  enableTorque(id);
}

static int32_t readPresentPositionTicks(int id) {
  uint32_t pos_u32 = 0;
  if (!read4B(id, ADDR_PRESENT_POSITION, pos_u32)) {
    throw std::runtime_error("Failed to read present position");
  }
  return (int32_t)pos_u32; // interpret as signed
}

static void writeGoalPositionTicks(int id, int32_t goal_tick) {
  write4B(id, ADDR_GOAL_POSITION, (uint32_t)goal_tick);
}

// ---------------- Command update ----------------
static void trySendCommand()
{
  // Need both sides at least once
  if (!g_have_left || !g_have_right) return;

  const ros::Time now = ros::Time::now();
  if ((now - g_last_cmd_time).toSec() < g_min_cmd_period) return;
  g_last_cmd_time = now;

  // average angle (deg) from data[4]
  const double target_deg = 0.5 * (g_left_angle_deg + g_right_angle_deg);

  // Optionally "zero" at start
  if (g_zero_on_start && !g_have_angle0) {
    g_angle0_deg = target_deg;
    g_have_angle0 = true;
    ROS_INFO("Captured angle0 = %.3f deg (from avg left/right data[4])", g_angle0_deg);
    return;
  }

  const double ddeg = g_zero_on_start ? (target_deg - g_angle0_deg) : target_deg;

  // Convert degrees -> ticks (multi-turn allowed)
  const double ticks_per_deg = (TICKS_PER_REV / 360.0);
  const double delta_ticks_d = g_angle_scale * ddeg * ticks_per_deg;

  int32_t goal_tick = (int32_t)llround((double)g_base_tick + (double)g_goal_offset_ticks + delta_ticks_d);

  if (g_use_limits) {
    goal_tick = clampI32(goal_tick, g_min_goal_tick, g_max_goal_tick);
  }

  writeGoalPositionTicks(g_dxl_id, goal_tick);

  ROS_INFO_THROTTLE(0.5,
    "left=%.2f deg right=%.2f deg avg=%.2f deg d=%.2f deg -> goal_tick=%d",
    g_left_angle_deg, g_right_angle_deg, target_deg, ddeg, (int)goal_tick);
}

// ---------------- Subscribers ----------------
static void leftCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_left: expected >= 7 elements, got %zu", msg->data.size());
    return;
  }
  g_left_angle_deg = msg->data[4];
  g_have_left = true;
  trySendCommand();
}

static void rightCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_right: expected >= 7 elements, got %zu", msg->data.size());
    return;
  }
  g_right_angle_deg = msg->data[4];
  g_have_right = true;
  trySendCommand();
}

// ---------------- main ----------------
int main(int argc, char **argv){
  ros::init(argc, argv, "angleavg_to_dynamixel_extpos");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string left_topic  = "/imu_data_left";
  std::string right_topic = "/imu_data_right";
  std::string port = DEVICENAME;

  pnh.param<std::string>("left_topic", left_topic, left_topic);
  pnh.param<std::string>("right_topic", right_topic, right_topic);
  pnh.param<std::string>("port", port, port);

  int baud = BAUDRATE;
  pnh.param<int>("baud", baud, BAUDRATE);

  pnh.param<int>("dxl_id", g_dxl_id, 1);

  pnh.param<bool>("zero_on_start", g_zero_on_start, true);
  pnh.param<double>("angle_scale", g_angle_scale, 20.0);
  pnh.param<int>("goal_offset_ticks", g_goal_offset_ticks, 0);

  pnh.param<bool>("use_limits", g_use_limits, true);
  int min_goal_tick = DEFAULT_MIN_GOAL_TICK;
  int max_goal_tick = DEFAULT_MAX_GOAL_TICK;
  pnh.param<int>("min_goal_tick", min_goal_tick, DEFAULT_MIN_GOAL_TICK);
  pnh.param<int>("max_goal_tick", max_goal_tick, DEFAULT_MAX_GOAL_TICK);
  g_min_goal_tick = (int32_t)min_goal_tick;
  g_max_goal_tick = (int32_t)max_goal_tick;

  pnh.param<double>("min_cmd_period", g_min_cmd_period, 0.01);

  // Init SDK
  portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open port: %s", port.c_str());
    return 1;
  }
  if (!portHandler->setBaudRate(baud)) {
    ROS_ERROR("Failed to set baudrate: %d", baud);
    return 1;
  }

  // Configure Dynamixel
  setOperatingMode(g_dxl_id, OPERATING_MODE_EXT_POS);

  // Base tick = current position
  try {
    g_base_tick = readPresentPositionTicks(g_dxl_id);
    ROS_INFO("DXL present position (base_tick) = %d", (int)g_base_tick);
  } catch (const std::exception& e) {
    ROS_ERROR("Error reading present position: %s", e.what());
    return 1;
  }

  ros::Subscriber subL = nh.subscribe(left_topic,  50, leftCallback);
  ros::Subscriber subR = nh.subscribe(right_topic, 50, rightCallback);

  ROS_INFO("angleavg_to_dynamixel_extpos running.");
  ROS_INFO("Left topic : %s", left_topic.c_str());
  ROS_INFO("Right topic: %s", right_topic.c_str());
  ROS_INFO("DXL: id=%d port=%s baud=%d mode=ExtendedPosition(4)", g_dxl_id, port.c_str(), baud);

  ros::spin();

  disableTorque(g_dxl_id);
  portHandler->closePort();
  return 0;
}
