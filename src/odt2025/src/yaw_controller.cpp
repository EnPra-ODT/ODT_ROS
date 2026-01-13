#include <iostream>
#include <cmath>
#include <string>
#include <stdexcept>
#include <array>

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

#define DEFAULT_MIN_GOAL_TICK   -1048575
#define DEFAULT_MAX_GOAL_TICK    1048575

// ---------------- Globals ----------------
dynamixel::PortHandler *portHandler = nullptr;
dynamixel::PacketHandler *packetHandler = nullptr;

static constexpr int kNumMotors = 4;
static const std::array<int, kNumMotors> g_ids = {1, 2, 3, 4};

static bool g_zero_on_start = true;
static bool g_have_angle0 = false;
static double g_angle0_deg = 0.0;

static double g_angle_scale = 20.0;         // gear ratio / mapping gain

static std::array<int32_t, kNumMotors> g_base_tick = {0, 0, 0, 0};
static std::array<int32_t, kNumMotors> g_goal_offset_ticks = {0, 0, 0, 0};

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

// ---- foot contact gating ----
// contact[i] == true means "freeze" motor i
static std::array<bool, kNumMotors> g_contact = {false, false, false, false};
static std::array<bool, kNumMotors> g_contact_prev = {false, false, false, false};
// hold tick when frozen
static std::array<int32_t, kNumMotors> g_hold_tick = {0, 0, 0, 0};
static bool g_have_contact = false;

// Sync write object
static dynamixel::GroupSyncWrite *g_syncWrite = nullptr;

// ---------------- Helpers ----------------
static inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  return std::max(lo, std::min(hi, v));
}

static inline void packInt32LE(int32_t v, uint8_t out[4]) {
  uint32_t u = (uint32_t)v;
  out[0] = (u >> 0) & 0xFF;
  out[1] = (u >> 8) & 0xFF;
  out[2] = (u >> 16) & 0xFF;
  out[3] = (u >> 24) & 0xFF;
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
  return (int32_t)pos_u32;
}

static bool syncWriteGoalPositions(const std::array<int32_t, kNumMotors>& goal_ticks) {
  if (!g_syncWrite) return false;

  g_syncWrite->clearParam();

  for (int i = 0; i < kNumMotors; i++) {
    uint8_t param[4];
    packInt32LE(goal_ticks[i], param);

    bool ok = g_syncWrite->addParam((uint8_t)g_ids[i], param);
    if (!ok) {
      ROS_ERROR("GroupSyncWrite addParam failed for id=%d", g_ids[i]);
      g_syncWrite->clearParam();
      return false;
    }
  }

  int comm = g_syncWrite->txPacket();
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("GroupSyncWrite txPacket failed: %s", packetHandler->getTxRxResult(comm));
    g_syncWrite->clearParam();
    return false;
  }

  g_syncWrite->clearParam();
  return true;
}

// ---------------- Core command update ----------------
static void trySendCommand(){
  if (!g_have_left || !g_have_right) return;
  if (!g_have_contact) return;

  const ros::Time now = ros::Time::now();
  if ((now - g_last_cmd_time).toSec() < g_min_cmd_period) return;
  g_last_cmd_time = now;

  const double target_deg = 0.5 * (g_left_angle_deg + g_right_angle_deg);

  if (g_zero_on_start && !g_have_angle0) {
    g_angle0_deg = target_deg;
    g_have_angle0 = true;
    ROS_INFO("Captured angle0 = %.3f deg (avg left/right data[4])", g_angle0_deg);
    return;
  }

  const double ddeg = g_zero_on_start ? (target_deg - g_angle0_deg) : target_deg;

  const double ticks_per_deg = (TICKS_PER_REV / 360.0);
  const double delta_ticks_d = g_angle_scale * ddeg * ticks_per_deg;
  const int32_t delta_ticks = (int32_t)llround(delta_ticks_d);

  std::array<int32_t, kNumMotors> goal_ticks;

  for (int i = 0; i < kNumMotors; i++) {
    if (g_contact[i]) {
      goal_ticks[i] = g_hold_tick[i];
      continue;
    }

    int64_t goal64 = (int64_t)g_base_tick[i] + (int64_t)g_goal_offset_ticks[i] + (int64_t)delta_ticks;
    int32_t goal = (int32_t)goal64;

    if (g_use_limits) goal = clampI32(goal, g_min_goal_tick, g_max_goal_tick);
    goal_ticks[i] = goal;
  }

  syncWriteGoalPositions(goal_ticks);

  ROS_INFO_THROTTLE(0.5,
    "avg=%.2f deg d=%.2f deg delta=%d | contact=[%d %d %d %d] goals=[%d %d %d %d]",
    target_deg, ddeg, (int)delta_ticks,
    (int)g_contact[0], (int)g_contact[1], (int)g_contact[2], (int)g_contact[3],
    (int)goal_ticks[0], (int)goal_ticks[1], (int)goal_ticks[2], (int)goal_ticks[3]);
}

// ---------------- Subscribers ----------------
static void leftCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_left: expected >= 7 elements, got %zu", msg->data.size());
    return;
  }
  g_left_angle_deg = msg->data[4];
  g_have_left = true;
  trySendCommand();
}

static void rightCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_right: expected >= 7 elements, got %zu", msg->data.size());
    return;
  }
  g_right_angle_deg = msg->data[4];
  g_have_right = true;
  trySendCommand();
}

static void footContactCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if (msg->data.size() < 4) {
    ROS_WARN_THROTTLE(1.0, "foot_contact_array: expected 4 elements, got %zu", msg->data.size());
    return;
  }

  g_have_contact = true;

  // Update contact flags and detect rising edges (0->1) to capture hold ticks immediately
  for (int i = 0; i < kNumMotors; i++) {
    const bool new_contact = (msg->data[i] >= 0.5);
    const bool old_contact = g_contact[i];

    g_contact_prev[i] = old_contact;
    g_contact[i] = new_contact;

    // Rising edge: 0 -> 1 : capture present position as hold tick + immediately command it
    if (!old_contact && new_contact) {
      try {
        g_hold_tick[i] = readPresentPositionTicks(g_ids[i]);
        ROS_INFO("Contact rising on id=%d: freeze at tick=%d", g_ids[i], (int)g_hold_tick[i]);
      } catch (const std::exception& e) {
        ROS_ERROR("Contact rising on id=%d but readPresentPosition failed: %s", g_ids[i], e.what());
        // fallback: hold base tick (not perfect, but prevents wild motion)
        g_hold_tick[i] = g_base_tick[i];
      }
    }
  }
  trySendCommand();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "angleavg_to_4dynamixel_extpos_with_contact_gate");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string left_topic  = "/imu_data_left";
  std::string right_topic = "/imu_data_right";
  std::string contact_topic = "/foot_contact_array";
  std::string port = DEVICENAME;

  pnh.param<std::string>("left_topic", left_topic, left_topic);
  pnh.param<std::string>("right_topic", right_topic, right_topic);
  pnh.param<std::string>("contact_topic", contact_topic, contact_topic);
  pnh.param<std::string>("port", port, port);

  int baud = BAUDRATE;
  pnh.param<int>("baud", baud, BAUDRATE);

  pnh.param<bool>("zero_on_start", g_zero_on_start, true);
  pnh.param<double>("angle_scale", g_angle_scale, 20.0);

  // per-motor offsets
  int off1=0, off2=0, off3=0, off4=0;
  pnh.param<int>("goal_offset_ticks_1", off1, 0);
  pnh.param<int>("goal_offset_ticks_2", off2, 0);
  pnh.param<int>("goal_offset_ticks_3", off3, 0);
  pnh.param<int>("goal_offset_ticks_4", off4, 0);
  g_goal_offset_ticks = {(int32_t)off1, (int32_t)off2, (int32_t)off3, (int32_t)off4};

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

  // Create GroupSyncWrite for Goal Position (4 bytes)
  g_syncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

  // Configure all 4 Dynamixels
  for (int i = 0; i < kNumMotors; i++) {
    disableTorque(g_ids[i]);
  }
  for (int i = 0; i < kNumMotors; i++) {
    setOperatingMode(g_ids[i], OPERATING_MODE_EXT_POS);
  }
  for (int i = 0; i < kNumMotors; i++) {
    enableTorque(g_ids[i]);
  }
  // Base tick for each motor + initialize hold ticks = base ticks
  try {
    for (int i = 0; i < kNumMotors; i++) {
      g_base_tick[i] = readPresentPositionTicks(g_ids[i]);
      g_hold_tick[i] = g_base_tick[i];
      ROS_INFO("DXL id=%d present position (base_tick) = %d", g_ids[i], (int)g_base_tick[i]);
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Error reading present positions: %s", e.what());
    return 1;
  }

  ros::Subscriber subL = nh.subscribe(left_topic,  50, leftCallback);
  ros::Subscriber subR = nh.subscribe(right_topic, 50, rightCallback);
  ros::Subscriber subC = nh.subscribe(contact_topic, 50, footContactCallback);

  ROS_INFO("Node running with contact gating.");
  ROS_INFO("Left   topic : %s", left_topic.c_str());
  ROS_INFO("Right  topic : %s", right_topic.c_str());
  ROS_INFO("Contact topic: %s", contact_topic.c_str());
  ROS_INFO("DXL IDs: 1..4 port=%s baud=%d mode=ExtendedPosition(4)", port.c_str(), baud);

  ros::spin();

  // Cleanup
  for (int i = 0; i < kNumMotors; i++) disableTorque(g_ids[i]);
  portHandler->closePort();

  delete g_syncWrite;
  g_syncWrite = nullptr;

  return 0;
}
