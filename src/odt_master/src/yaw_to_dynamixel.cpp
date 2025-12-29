#include <iostream>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
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
static bool g_have_yaw0 = false;
static double g_yaw0_deg = 0.0;

static double g_yaw_scale = 20.0;            // set -1.0 to invert, currently 20:1 Gear Ratio
static int32_t g_base_tick = 0;             // motor present position at start
static int32_t g_goal_offset_ticks = 0;     // extra offset ticks

static bool g_use_limits = true;
static int32_t g_min_goal_tick = DEFAULT_MIN_GOAL_TICK;
static int32_t g_max_goal_tick = DEFAULT_MAX_GOAL_TICK;

static double g_min_cmd_period = 0.01;      // seconds
static ros::Time g_last_cmd_time(0);

// --------- Yaw unwrap state (NEW) ---------
static bool g_have_yaw_prev = false;
static double g_yaw_prev_deg = 0.0;
static double g_yaw_unwrapped_deg = 0.0;

// ---------------- Helpers ----------------
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

static inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  return std::max(lo, std::min(hi, v));
}

// Unwrap yaw in degrees to be continuous (NEW)
static double unwrapYawDeg(double yaw_deg_wrapped)
{
  if (!g_have_yaw_prev) {
    g_have_yaw_prev = true;
    g_yaw_prev_deg = yaw_deg_wrapped;
    g_yaw_unwrapped_deg = yaw_deg_wrapped;
    return g_yaw_unwrapped_deg;
  }

  double delta = yaw_deg_wrapped - g_yaw_prev_deg;

  // choose the shortest step across the -180/180 boundary
  if (delta > 180.0)  delta -= 360.0;
  if (delta < -180.0) delta += 360.0;

  g_yaw_unwrapped_deg += delta;
  g_yaw_prev_deg = yaw_deg_wrapped;
  return g_yaw_unwrapped_deg;
}

//q*v*q^-1
static void quatToRPY_ZYX(double qw, double qx, double qy, double qz,double &roll, double &pitch, double &yaw){
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::fabs(sinp) >= 1.0) pitch = std::copysign(M_PI / 2.0, sinp);
  else                        pitch = std::asin(sinp);

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
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
  // Interpret as signed int32
  return (int32_t)pos_u32;
}

static void writeGoalPositionTicks(int id, int32_t goal_tick) {
  // Extended position goal is signed int32, but SDK takes uint32_t (bitwise same)
  write4B(id, ADDR_GOAL_POSITION, (uint32_t)goal_tick);
}

// ---------------- IMU callback ----------------
static void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  const ros::Time now = ros::Time::now();
  if ((now - g_last_cmd_time).toSec() < g_min_cmd_period) return;
  g_last_cmd_time = now;

  // Quaternion -> yaw (rad)
  double roll=0, pitch=0, yaw=0;
  quatToRPY_ZYX(msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                roll, pitch, yaw);

  // Most IMUs output wrapped yaw in [-180, 180] after this conversion
  const double yaw_deg_wrapped = rad2deg(yaw);

  // NEW: make yaw continuous (no jumps at +/-180)
  const double yaw_deg = unwrapYawDeg(yaw_deg_wrapped);

  // Capture yaw0 once (optional) using UNWRAPPED yaw
  if (g_zero_on_start && !g_have_yaw0) {
    g_yaw0_deg = yaw_deg;
    g_have_yaw0 = true;
    ROS_INFO("Captured IMU yaw0 (unwrapped) = %.3f deg", g_yaw0_deg);
    return;
  }

  // Extended mode: now dyaw is continuous, so no sudden jump
  const double dyaw_deg = g_zero_on_start ? (yaw_deg - g_yaw0_deg) : yaw_deg;

  // Convert degrees -> ticks (multi-turn allowed)
  const double ticks_per_deg = (TICKS_PER_REV / 360.0);
  const double delta_ticks_d = g_yaw_scale * dyaw_deg * ticks_per_deg;

  int32_t goal_tick = (int32_t)llround((double)g_base_tick + (double)g_goal_offset_ticks + delta_ticks_d);

  if (g_use_limits) {
    goal_tick = clampI32(goal_tick, g_min_goal_tick, g_max_goal_tick);
  }

  writeGoalPositionTicks(g_dxl_id, goal_tick);

  ROS_INFO_THROTTLE(0.5,
    "yaw_wrapped=%.2f deg, yaw_unwrapped=%.2f deg, dyaw=%.2f deg -> goal_tick=%d",
    yaw_deg_wrapped, yaw_deg, dyaw_deg, (int)goal_tick);
}

// ---------------- main ----------------
int main(int argc, char **argv){
  ros::init(argc, argv, "yaw_to_dynamixel_extpos");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string imu_topic = "/imu";
  std::string port = DEVICENAME;

  pnh.param<std::string>("imu_topic", imu_topic, imu_topic);
  pnh.param<std::string>("port", port, port);

  int baud = BAUDRATE;
  pnh.param<int>("baud", baud, BAUDRATE);

  pnh.param<int>("dxl_id", g_dxl_id, 1);

  pnh.param<bool>("zero_on_start", g_zero_on_start, true);
  pnh.param<double>("yaw_scale", g_yaw_scale, 20.0);
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

  ros::Subscriber sub = nh.subscribe(imu_topic, 50, imuCallback);

  ROS_INFO("yaw_to_dynamixel_extpos running.");
  ROS_INFO("IMU topic: %s", imu_topic.c_str());
  ROS_INFO("DXL: id=%d port=%s baud=%d mode=ExtendedPosition(4)", g_dxl_id, port.c_str(), baud);

  ros::spin();

  disableTorque(g_dxl_id);
  portHandler->closePort();
  return 0;
}
