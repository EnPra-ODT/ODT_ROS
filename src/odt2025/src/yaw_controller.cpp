/*yaw_controller.cpp
 * Yaw Controller Node
 * 
 * PURPOSE:
 *   Controls Dynamixel servo motors based on IMU yaw (rotation) data.
 *   Implements contact-based freezing: when a foot makes contact with the ground,
 *   the corresponding motor position is frozen to prevent unwanted movement.
 * 
 * SUBSCRIBES TO:
 *   - /imu_data_left (std_msgs/Float64MultiArray) [default, configurable via "left_topic" param]
 *     Format: [timestamp_ms, ax, ay, az, roll, pitch, yaw]
 *     - Used to read left foot roll angle (data[4])
 * 
 *   - /imu_data_right (std_msgs/Float64MultiArray) [default, configurable via "right_topic" param]
 *     Format: [timestamp_ms, ax, ay, az, roll, pitch, yaw]
 *     - Used to read right foot roll angle (data[4]) - this is the primary control signal
 * 
 *   - /foot_contact_pair (std_msgs/Float64MultiArray) [default, configurable via "contact_topic" param]
 *     Format: [contact_motor1, contact_motor2, contact_motor3, contact_motor4]
 *     - Each element is 0.0 (no contact) or 1.0 (contact detected)
 *     - Maps to motor IDs 1-4 by index
 * 
 * PUBLISHES:
 *   None (directly controls Dynamixel motors via serial)
 * 
 * FEATURES:
 *   - Dynamic motor configuration: supports any subset of motors (not just 1-4)
 *   - Zero-on-start: optionally captures initial angle and works with deltas
 *   - Contact-based position freezing: motors hold position when foot contacts ground
 *   - Configurable angle scaling for fine-tuning control sensitivity
 *   - Position limits to prevent over-rotation
 *   - Rate limiting to prevent excessive motor commands
 * 
 * MOTOR CONTROL LOGIC:
 *   1. Reads right IMU roll angle as primary control input
 *   2. Optionally zeros the angle on first reading
 *   3. Scales the angle and converts to motor ticks
 *   4. For each motor:
 *      - If in contact: freeze at last commanded position
 *      - If not in contact: move to scaled target position
 *   5. Uses GroupSyncWrite for efficient multi-motor control
 * 
 * KEY PARAMETERS (ROS params):
 *   - connected_ids: List of Dynamixel IDs to control (e.g., [1,2,3,4] or [1,3])
 *   - goal_offset_ticks: Per-motor offset to apply to goal positions
 *   - zero_on_start: If true, zero angle on first reading (default: true)
 *   - angle_scale: Multiplier for angle-to-tick conversion (default: 20.0)
 *   - use_limits: Enable position limits (default: true)
 *   - min_goal_tick/max_goal_tick: Position limits in ticks
 *   - min_cmd_period: Minimum time between motor commands (seconds, default: 0.01)
 *   - port: Serial port for Dynamixel communication (default: /dev/ttyUSB0)
 *   - baud: Baudrate for serial communication (default: 1000000)
 */

#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <vector>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

// -------------------- Dynamixel Register Addresses --------------------
static constexpr int ADDR_OPERATING_MODE   = 11;   // Operating mode (position/velocity/etc)
static constexpr int ADDR_TORQUE_ENABLE    = 64;   // Enable/disable motor torque
static constexpr int ADDR_GOAL_POSITION    = 116;  // Target position to move to
static constexpr int ADDR_PRESENT_POSITION = 132;  // Current position readback

static constexpr float PROTOCOL_VERSION = 2.0f;

// -------------------- Serial Communication Settings --------------------
static constexpr const char* DEVICENAME = "/dev/ttyUSB0";
static constexpr int   BAUDRATE         = 1000000;

// -------------------- Motor Constants --------------------
static constexpr double TICKS_PER_REV   = 4096.0;   // Encoder resolution (X-series)
static constexpr uint8_t OPERATING_MODE_EXT_POS = 4; // Extended position mode (multi-turn)

static constexpr uint8_t TORQUE_ENABLE  = 1;
static constexpr uint8_t TORQUE_DISABLE = 0;

// Default position limits (very wide range for extended position mode)
static constexpr int32_t DEFAULT_MIN_GOAL_TICK = -1048575;
static constexpr int32_t DEFAULT_MAX_GOAL_TICK =  1048575;

// -------------------- Dynamixel SDK Handles --------------------
static dynamixel::PortHandler*    g_port  = nullptr;
static dynamixel::PacketHandler*  g_pkt   = nullptr;
static dynamixel::GroupSyncWrite* g_sync  = nullptr;

// -------------------- Motor Configuration (Dynamic) --------------------
static std::vector<int> g_ids;                      // Connected motor IDs (e.g., {1,2,3,4} or {1,3})
static std::vector<int32_t> g_base_tick;            // Initial position of each motor
static std::vector<int32_t> g_goal_offset_ticks;    // Per-motor offsets to apply
static std::vector<bool>    g_contact;              // Contact state for each motor
static std::vector<int32_t> g_hold_tick;            // Position to freeze at when in contact
static bool                 g_have_last_goal = false;
static std::vector<int32_t> g_last_goal_tick;       // Last commanded position

// -------------------- Control Parameters --------------------
static bool   g_zero_on_start = true;               // Zero angle on first reading
static bool   g_have_angle0   = false;
static double g_angle0_deg    = 0.0;                // Initial angle for zeroing

static double g_angle_scale   = 20.0;               // Scaling factor for angle-to-tick conversion

static bool    g_use_limits     = true;             // Enable position limits
static int32_t g_min_goal_tick = DEFAULT_MIN_GOAL_TICK;
static int32_t g_max_goal_tick = DEFAULT_MAX_GOAL_TICK;

static double   g_min_cmd_period = 0.01;            // Minimum time between commands (seconds)
static ros::Time g_last_cmd_time(0);

// -------------------- IMU Data State --------------------
static bool   g_have_left  = false;
static bool   g_have_right = false;
static double g_left_deg   = 0.0;                   // Left IMU roll angle
static double g_right_deg  = 0.0;                   // Right IMU roll angle (primary control)

static bool g_have_contact = false;                 // Have we received contact data yet

// -------------------- Helper Functions --------------------

// Clamp integer value to range
static inline int32_t clampInt32(int32_t v, int32_t lo, int32_t hi) {
  return std::max(lo, std::min(hi, v));
}

// Pack 32-bit integer into byte array (little-endian)
static inline void packInt32LE(int32_t v, uint8_t out[4]) {
  uint32_t u = static_cast<uint32_t>(v);
  out[0] = static_cast<uint8_t>((u >> 0)  & 0xFF);
  out[1] = static_cast<uint8_t>((u >> 8)  & 0xFF);
  out[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  out[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

// Write single byte to Dynamixel register
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

// Read 4 bytes from Dynamixel register
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

// -------------------- Dynamixel Control Functions --------------------

// Enable or disable motor torque
static void setTorque(int id, bool on) {
  (void)write1B(id, ADDR_TORQUE_ENABLE, on ? TORQUE_ENABLE : TORQUE_DISABLE);
}

// Set motor to extended position mode (allows multi-turn rotation)
static void setOperatingModeExtPos(int id) {
  setTorque(id, false);  // Must disable torque to change mode
  (void)write1B(id, ADDR_OPERATING_MODE, OPERATING_MODE_EXT_POS);
  setTorque(id, true);   // Re-enable torque
}

// Read current motor position
static int32_t readPresentPositionTicks(int id) {
  uint32_t pos_u32 = 0;
  if (!read4B(id, ADDR_PRESENT_POSITION, pos_u32)) {
    throw std::runtime_error("Failed to read present position");
  }
  return static_cast<int32_t>(pos_u32);
}

// Send goal positions to all motors simultaneously (efficient bulk write)
static bool syncWriteGoalPositions(const std::vector<int32_t>& goals) {
  if (!g_sync) return false;

  g_sync->clearParam();

  // Add each motor's goal to the sync write
  for (size_t i = 0; i < g_ids.size(); i++) {
    uint8_t param[4];
    packInt32LE(goals[i], param);

    if (!g_sync->addParam(static_cast<uint8_t>(g_ids[i]), param)) {
      ROS_ERROR("GroupSyncWrite addParam failed for id=%d", g_ids[i]);
      g_sync->clearParam();
      return false;
    }
  }

  // Transmit all goal positions in one packet
  int comm = g_sync->txPacket();
  if (comm != COMM_SUCCESS) {
    ROS_ERROR("GroupSyncWrite txPacket failed: %s", g_pkt->getTxRxResult(comm));
    g_sync->clearParam();
    return false;
  }

  g_sync->clearParam();
  return true;
}

// -------------------- Main Control Logic --------------------

static inline double ticksToDeg(int32_t ticks)
{
  return ticks * (360.0 / 4096.0);
}

// Compute and send motor commands based on current state
static void trySendCommand() {
  // Wait until we have all required data
  if (!g_have_right || !g_have_contact) return;
  if (g_ids.empty()) return;

  // Rate limiting: don't send commands too frequently
  const ros::Time now = ros::Time::now();
  if ((now - g_last_cmd_time).toSec() < g_min_cmd_period) return;
  g_last_cmd_time = now;

  const double target_deg = -1.0 * g_right_deg;  // Use right IMU as primary control

  // ---- Zero angle on first reading ----
  if (g_zero_on_start && !g_have_angle0) {
    g_angle0_deg = target_deg;
    g_have_angle0 = true;
    ROS_INFO("Captured angle0=%.3f deg (right IMU data[4])", g_angle0_deg);
    return;
  }

  // ---- Calculate target position ----
  const double ddeg = g_zero_on_start ? (target_deg - g_angle0_deg) : target_deg;
  const double ticks_per_deg = (TICKS_PER_REV / 360.0);
  const int32_t delta_ticks  = static_cast<int32_t>(llround(g_angle_scale * ddeg * ticks_per_deg));

  std::vector<int32_t> goals(g_ids.size(), 0);

  // ---- Compute goal for each motor ----
  for (size_t i = 0; i < g_ids.size(); i++) {
    // If motor is in contact, freeze at hold position
    if (g_contact[i]) {
      goals[i] = g_hold_tick[i];
      continue;
    }

    // Otherwise, move to target position
    int64_t goal64 = static_cast<int64_t>(g_base_tick[i])
                   + static_cast<int64_t>(g_goal_offset_ticks[i])
                   + static_cast<int64_t>(delta_ticks);

    int32_t goal = static_cast<int32_t>(goal64);
    if (g_use_limits) goal = clampInt32(goal, g_min_goal_tick, g_max_goal_tick);
    goals[i] = goal;
  }

  // Store goals for contact freezing logic
  g_last_goal_tick = goals;
  g_have_last_goal = true;

  // Send commands to motors
  (void)syncWriteGoalPositions(goals);

  // ---- Debug output ----
 std::ostringstream oss;
  oss << "avg=" << std::fixed << std::setprecision(2) << target_deg
      << " d=" << ddeg << " delta=" << delta_ticks
      << " | ids=[";

  for (size_t i = 0; i < g_ids.size(); i++) {
    oss << g_ids[i] << (i + 1 < g_ids.size() ? " " : "");
  }

  oss << "] contact=[";
  for (size_t i = 0; i < g_contact.size(); i++) {
    oss << (g_contact[i] ? 1 : 0)
        << (i + 1 < g_contact.size() ? " " : "");
  }

  oss << "] goals_deg=[";
  for (size_t i = 0; i < goals.size(); i++) {
    const double goal_deg = ticksToDeg(goals[i]);
    oss << std::fixed << std::setprecision(2)
        << goal_deg
        << (i + 1 < goals.size() ? " " : "");
  }
  oss << "]";

  ROS_INFO_THROTTLE(0.5, "%s", oss.str().c_str());

}

// -------------------- ROS Callbacks --------------------

// Callback for left IMU data
static void leftCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_left: expected >=7 elements, got %zu", msg->data.size());
    return;
  }
  g_left_deg = msg->data[4];  // Extract roll angle
  g_have_left = true;
  trySendCommand();
}

// Callback for right IMU data (primary control signal)
static void rightCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 7) {
    ROS_WARN_THROTTLE(1.0, "imu_data_right: expected >=7 elements, got %zu", msg->data.size());
    return;
  }
  g_right_deg = msg->data[4];  // Extract roll angle
  g_have_right = true;
  trySendCommand();
}

// Callback for foot contact data
// Detects contact transitions and freezes motor positions accordingly
static void footContactCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 4) {
    ROS_WARN_THROTTLE(1.0, "foot_contact_pair: expected 4 elements, got %zu", msg->data.size());
    return;
  }
  g_have_contact = true;

  // Process contact state for each motor
  for (size_t i = 0; i < g_ids.size(); i++) {
    const int id = g_ids[i];
    const int idx = id - 1;  // Map motor ID to message index (assumes IDs 1-4)
    
    bool new_contact = false;
    if (0 <= idx && idx < 4) {
      new_contact = (msg->data[idx] >= 0.5);  // Threshold for contact detection
    }

    const bool old_contact = g_contact[i];
    g_contact[i] = new_contact;

    // ---- Detect contact rising edge (foot just made contact) ----
    if (!old_contact && new_contact) {
      // Freeze at last commanded position if available
      if (g_have_last_goal && i < g_last_goal_tick.size()) {
        g_hold_tick[i] = g_last_goal_tick[i];
        ROS_INFO("Contact rising id=%d: freeze at LAST GOAL tick=%d", id, (int)g_hold_tick[i]);
      } else {
        // Fall back to base position if no commands sent yet
        g_hold_tick[i] = g_base_tick[i];
        ROS_WARN("Contact rising id=%d: no last goal yet; freeze at base_tick=%d", id, (int)g_hold_tick[i]);
      }
    }
  }

  // Reset rate limiter to allow immediate command after contact change
  g_last_cmd_time = ros::Time(0);
  trySendCommand();
}

// -------------------- Main --------------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "yaw_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // ---- Load ROS parameters ----
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

  pnh.param<bool>("use_limits", g_use_limits, true);
  pnh.param<int32_t>("min_goal_tick", g_min_goal_tick, DEFAULT_MIN_GOAL_TICK);
  pnh.param<int32_t>("max_goal_tick", g_max_goal_tick, DEFAULT_MAX_GOAL_TICK);

  pnh.param<double>("min_cmd_period", g_min_cmd_period, 0.01);

  // ---- Load connected motor IDs ----
  std::vector<int> connected_ids;
  if (!pnh.getParam("connected_ids", connected_ids) || connected_ids.empty()) {
    connected_ids = {1,2,3,4};  // Default to motor 4 only
  }
  
  // Sort and remove duplicates
  std::sort(connected_ids.begin(), connected_ids.end());
  connected_ids.erase(std::unique(connected_ids.begin(), connected_ids.end()), connected_ids.end());

  // Validate motor IDs (Dynamixel allows 1-252)
  for (int id : connected_ids) {
    if (id < 1 || id > 252) {
      ROS_ERROR("Invalid id in connected_ids: %d", id);
      return 1;
    }
  }

  g_ids = connected_ids;

  // ---- Load per-motor position offsets ----
  std::vector<int> offsets;
  if (!pnh.getParam("goal_offset_ticks", offsets) || offsets.empty()) {
    offsets.assign(g_ids.size(), 0);
  }
  if (offsets.size() != g_ids.size()) {
    ROS_WARN("goal_offset_ticks size (%zu) != connected_ids size (%zu). Using zeros.",
             offsets.size(), g_ids.size());
    offsets.assign(g_ids.size(), 0);
  }

  g_goal_offset_ticks.resize(g_ids.size(), 0);
  for (size_t i = 0; i < g_ids.size(); i++) {
    g_goal_offset_ticks[i] = static_cast<int32_t>(offsets[i]);
  }

  // ---- Allocate per-motor state arrays ----
  g_base_tick.assign(g_ids.size(), 0);
  g_hold_tick.assign(g_ids.size(), 0);
  g_contact.assign(g_ids.size(), false);
  g_last_goal_tick.assign(g_ids.size(), 0);

  // ---- Initialize Dynamixel communication ----
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

  // Create sync write handler for efficient bulk commands
  g_sync = new dynamixel::GroupSyncWrite(g_port, g_pkt, ADDR_GOAL_POSITION, 4);

  // ---- Configure each motor ----
  for (size_t i = 0; i < g_ids.size(); i++) {
    setOperatingModeExtPos(g_ids[i]);
  }

  // ---- Read initial positions ----
  try {
    for (size_t i = 0; i < g_ids.size(); i++) {
      g_base_tick[i] = readPresentPositionTicks(g_ids[i]);
      g_hold_tick[i] = g_base_tick[i];
      ROS_INFO("DXL id=%d base_tick=%d", g_ids[i], (int)g_base_tick[i]);
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Error reading present positions: %s", e.what());
    return 1;
  }

  // ---- Set up ROS subscribers ----
  ros::Subscriber subL = nh.subscribe(left_topic,    50, leftCallback);
  ros::Subscriber subR = nh.subscribe(right_topic,   50, rightCallback);
  ros::Subscriber subC = nh.subscribe(contact_topic, 50, footContactCallback);

  ROS_INFO("yaw_controller running.");
  ROS_INFO("Left: %s  Right: %s  Contact: %s", 
           left_topic.c_str(), right_topic.c_str(), contact_topic.c_str());
  ROS_INFO("Port: %s baud=%d connected_ids size=%zu", port.c_str(), baud, g_ids.size());

  ros::spin();

  // ---- Cleanup on shutdown ----
  for (size_t i = 0; i < g_ids.size(); i++) {
    setTorque(g_ids[i], false);
  }
  g_port->closePort();
  delete g_sync;
  g_sync = nullptr;

  return 0;
}