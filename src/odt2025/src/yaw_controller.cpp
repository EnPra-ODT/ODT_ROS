#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

// ---------------- Dynamixel ----------------
static constexpr int ADDR_OPERATING_MODE   = 11;
static constexpr int ADDR_TORQUE_ENABLE    = 64;
static constexpr int ADDR_GOAL_POSITION    = 116;
static constexpr int ADDR_PRESENT_POSITION = 132;

static constexpr float PROTOCOL_VERSION = 2.0f;


static constexpr const char* DEVICENAME = "/dev/ttyUSB0";
static constexpr int   BAUDRATE         = 1000000;

static constexpr double TICKS_PER_REV   = 4096.0;   // X-series
static constexpr uint8_t OPERATING_MODE_EXT_POS = 4;

static constexpr uint8_t TORQUE_ENABLE  = 1;
static constexpr uint8_t TORQUE_DISABLE = 0;

static constexpr int32_t DEFAULT_MIN_GOAL_TICK = -1048575;
static constexpr int32_t DEFAULT_MAX_GOAL_TICK =  1048575;

static dynamixel::PortHandler*   g_port  = nullptr;
static dynamixel::PacketHandler* g_pkt   = nullptr;
static dynamixel::GroupSyncWrite* g_sync = nullptr;

static constexpr int kNumMotors = 4;
static const std::array<int, kNumMotors> g_ids = {1, 2, 3, 4};

// -------------------- Calculation -------------------- //
static bool   g_zero_on_start = true;
static bool   g_have_angle0   = false;
static double g_angle0_deg    = 0.0;

static double g_angle_scale   = 20.0;

static std::array<int32_t, kNumMotors> g_base_tick = {0, 0, 0, 0};
static std::array<int32_t, kNumMotors> g_goal_offset_ticks = {0, 0, 0, 0};

static bool   g_use_limits     = true;
static int32_t g_min_goal_tick = DEFAULT_MIN_GOAL_TICK;
static int32_t g_max_goal_tick = DEFAULT_MAX_GOAL_TICK;

static double   g_min_cmd_period = 0.01;
static ros::Time g_last_cmd_time(0);

static bool   g_have_left  = false;
static bool   g_have_right = false;
static double g_left_deg   = 0.0;
static double g_right_deg  = 0.0;

static bool g_have_contact = false;
static std::array<bool,   kNumMotors>  g_contact = {false, false, false, false};
static std::array<int32_t,kNumMotors>  g_hold_tick = {0, 0, 0, 0};

static bool g_have_last_goal = false;
static std::array<int32_t, kNumMotors> g_last_goal_tick = {0, 0, 0, 0};

// -------------------- Bit Encoder & Sender -------------------- //
static inline int32_t clampInt32(int32_t v, int32_t lo, int32_t hi) {
  return std::max(lo, std::min(hi, v));
}

static inline void packInt32LE(int32_t v, uint8_t out[4]) {
  uint32_t u = static_cast<uint32_t>(v);
  out[0] = static_cast<uint8_t>((u >> 0)  & 0xFF);
  out[1] = static_cast<uint8_t>((u >> 8)  & 0xFF);
  out[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  out[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

static bool write1B(int id, int addr, uint8_t val) {
  uint8_t dxl_error = 0;
  int comm = g_pkt->write1ByteTxRx(g_port, id, addr, val, &dxl_error);
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("DXL write1B failed (id=%d addr=%d): %s", id, addr, g_pkt->getTxRxResult(comm));
    return false;
  }
  if (dxl_error != 0) {
    ROS_ERROR("DXL write1B error (id=%d addr=%d): %s", id, addr, g_pkt->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

static bool read4B(int id, int addr, uint32_t &out) {
  uint8_t dxl_error = 0;
  int comm = g_pkt->read4ByteTxRx(g_port, id, addr, &out, &dxl_error);
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("DXL read4B failed (id=%d addr=%d): %s", id, addr, g_pkt->getTxRxResult(comm));
    return false;
  }
  if (dxl_error != 0) {
    ROS_ERROR("DXL read4B error (id=%d addr=%d): %s", id, addr, g_pkt->getRxPacketError(dxl_error));
    return false;
  }
  return true;
}

// ------------------- Dynamixel Functions ------------------- //
static void setTorque(int id, bool on) {
  write1B(id, ADDR_TORQUE_ENABLE, on ? TORQUE_ENABLE : TORQUE_DISABLE);
}

static void setOperatingModeExtPos(int id) {
  setTorque(id, false);
  write1B(id, ADDR_OPERATING_MODE, OPERATING_MODE_EXT_POS);
  setTorque(id, true);
}

static int32_t readPresentPositionTicks(int id) {
  uint32_t pos_u32 = 0;
  if (!read4B(id, ADDR_PRESENT_POSITION, pos_u32)) {
    throw std::runtime_error("Failed to read present position");
  }
  return static_cast<int32_t>(pos_u32);
}

static bool syncWriteGoalPositions(const std::array<int32_t, kNumMotors>& goals) {
  if (!g_sync) return false;

  g_sync->clearParam();

  for (int i = 0; i < kNumMotors; i++) {
    uint8_t param[4];
    packInt32LE(goals[i], param);

    if (!g_sync->addParam(static_cast<uint8_t>(g_ids[i]), param)) {
      ROS_ERROR("GroupSyncWrite addParam failed for id=%d", g_ids[i]);
      g_sync->clearParam();
      return false;
    }
  }

  int comm = g_sync->txPacket();
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("GroupSyncWrite txPacket failed: %s", g_pkt->getTxRxResult(comm));
    g_sync->clearParam();
    return false;
  }

  g_sync->clearParam();
  return true;
}

static void trySendCommand() {
  if ((!g_have_left || !g_have_right) || !g_have_contact) return; 

  const ros::Time now = ros::Time::now();
  if ((now - g_last_cmd_time).toSec() < g_min_cmd_period) return;
  g_last_cmd_time = now;

  const double target_deg = 0.5 * (g_left_deg + g_right_deg); //Target Yaw Degree is average of left and right

  if (g_zero_on_start && !g_have_angle0) {
    g_angle0_deg = target_deg;
    g_have_angle0 = true;
    ROS_INFO("Captured angle0=%.3f deg (avg left/right data[4])", g_angle0_deg);
    return;
  }

  const double ddeg = g_zero_on_start ? (target_deg - g_angle0_deg) : target_deg;
  const double ticks_per_deg = (TICKS_PER_REV / 360.0);
  const int32_t delta_ticks  = static_cast<int32_t>(llround(g_angle_scale * ddeg * ticks_per_deg));

  std::array<int32_t, kNumMotors> goals;

  for (int i = 0; i < kNumMotors; i++) {
    if (g_contact[i]) {
      goals[i] = g_hold_tick[i];
      continue;
    }

    int64_t goal64 = static_cast<int64_t>(g_base_tick[i]) + static_cast<int64_t>(g_goal_offset_ticks[i]) + static_cast<int64_t>(delta_ticks);
    int32_t goal = static_cast<int32_t>(goal64);

    if (g_use_limits){goal = clampInt32(goal, g_min_goal_tick, g_max_goal_tick);}
    goals[i] = goal;
  }

  g_last_goal_tick = goals;
  g_have_last_goal = true;

  (void)syncWriteGoalPositions(goals);

  ROS_INFO_THROTTLE(0.5, "avg=%.2f d=%.2f delta=%d | contact=[%d %d %d %d] goals=[%d %d %d %d]", target_deg, ddeg, (int)delta_ticks, (int)g_contact[0], (int)g_contact[1], (int)g_contact[2], (int)g_contact[3], (int)goals[0], (int)goals[1], (int)goals[2], (int)goals[3]);
}

// ----------------------- ROS ----------------------- // 
static void leftCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_left: expected >=7 elements, got %zu", msg->data.size());
    return;
  }
  g_left_deg = msg->data[4];
  g_have_left = true;
  trySendCommand();
}

static void rightCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_right: expected >=7 elements, got %zu", msg->data.size());
    return;
  }
  g_right_deg = msg->data[4];
  g_have_right = true;
  trySendCommand();
}

static void footContactCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 4) {
    ROS_WARN_THROTTLE(1.0, "foot_contact_pair: expected 4 elements, got %zu", msg->data.size());
    return;
  }

  g_have_contact = true;

  for (int i = 0; i < kNumMotors; i++) {
    const bool new_contact = (msg->data[i] >= 0.5);
    const bool old_contact = g_contact[i];
    g_contact[i] = new_contact;

    if (!old_contact && new_contact) {
      if (g_have_last_goal) {
        g_hold_tick[i] = g_last_goal_tick[i];
        ROS_INFO("Contact rising id=%d: freeze at LAST GOAL tick=%d", g_ids[i], (int)g_hold_tick[i]);
      } else {
        g_hold_tick[i] = g_base_tick[i];
        ROS_WARN("Contact rising id=%d: no last goal yet; freeze at base_tick=%d", g_ids[i], (int)g_hold_tick[i]);
      }
    }
  }

  g_last_cmd_time = ros::Time(0);
  trySendCommand();
}

// ---------------- main ----------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "yaw_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string left_topic   = "/imu_data_left";
  std::string right_topic  = "/imu_data_right";
  std::string contact_topic= "/foot_contact_pair";
  std::string port         = DEVICENAME;

  pnh.param<std::string>("left_topic", left_topic, left_topic);
  pnh.param<std::string>("right_topic", right_topic, right_topic);
  pnh.param<std::string>("contact_topic", contact_topic, contact_topic);
  pnh.param<std::string>("port", port, port);

  int baud = BAUDRATE;
  pnh.param<int>("baud", baud, BAUDRATE);

  pnh.param<bool>("zero_on_start", g_zero_on_start, true);
  pnh.param<double>("angle_scale", g_angle_scale, 20.0);

  int off1=0, off2=0, off3=0, off4=0;
  pnh.param<int>("goal_offset_ticks_1", off1, 0);
  pnh.param<int>("goal_offset_ticks_2", off2, 0);
  pnh.param<int>("goal_offset_ticks_3", off3, 0);
  pnh.param<int>("goal_offset_ticks_4", off4, 0);
  g_goal_offset_ticks = { (int32_t)off1, (int32_t)off2, (int32_t)off3, (int32_t)off4 };

  pnh.param<bool>("use_limits", g_use_limits, true);
  pnh.param<int32_t>("min_goal_tick", g_min_goal_tick, DEFAULT_MIN_GOAL_TICK);
  pnh.param<int32_t>("max_goal_tick", g_max_goal_tick, DEFAULT_MAX_GOAL_TICK);

  pnh.param<double>("min_cmd_period", g_min_cmd_period, 0.01);

  //Dynamixel Init
  g_port = dynamixel::PortHandler::getPortHandler(port.c_str());
  g_pkt  = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  if (!g_port->openPort()) {
    ROS_ERROR("Failed to open port: %s", port.c_str());
    return 1;
  }
  if (!g_port->setBaudRate(baud)) {
    ROS_ERROR("Failed to set baudrate: %d", baud);
    return 1;
  }
  g_sync = new dynamixel::GroupSyncWrite(g_port, g_pkt, ADDR_GOAL_POSITION, 4);
  for (int i = 0; i < kNumMotors; i++) {
    setOperatingModeExtPos(g_ids[i]);
  }
  try {//Read initial angle
    for (int i = 0; i < kNumMotors; i++) {
      g_base_tick[i] = readPresentPositionTicks(g_ids[i]);
      g_hold_tick[i] = g_base_tick[i];
      ROS_INFO("DXL id=%d base_tick=%d", g_ids[i], (int)g_base_tick[i]);
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Error reading present positions: %s", e.what());
    return 1;
  }

  ros::Subscriber subL = nh.subscribe(left_topic,   50, leftCallback);
  ros::Subscriber subR = nh.subscribe(right_topic,  50, rightCallback);
  ros::Subscriber subC = nh.subscribe(contact_topic,50, footContactCallback);

  ROS_INFO("yaw_controller running.");
  ROS_INFO("Left: %s  Right: %s  Contact: %s", left_topic.c_str(), right_topic.c_str(), contact_topic.c_str());
  ROS_INFO("Port: %s baud=%d IDs=1..4", port.c_str(), baud);

  ros::spin();

  for (int i = 0; i < kNumMotors; i++) setTorque(g_ids[i], false);
  g_port->closePort();
  delete g_sync;
  g_sync = nullptr;

  return 0;
}
