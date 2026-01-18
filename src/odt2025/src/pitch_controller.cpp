// kinco_speed_from_vmag_active.cpp
//
// Subscribes to /v_mag_active (cm/s) and commands Kinco CANopen drives (Profile Velocity mode)
// to match the speed in real-time.
//
// Converts: cm/s -> m/s -> roller RPM (roller dia) -> motor RPM (gear ratio) -> drive units -> 0x60FF
//
// Notes:
// - /v_mag_active is speed magnitude, so this code commands positive velocity only.
// - Uses a timer to refresh commands at cmd_refresh_hz (avoids blocking CAN writes in callbacks).
//
// Params (~):
//   can_iface            (string)  default "can0"
//   node_ids             (list)    default [1]
//   encoder_res          (int)     default 65536
//   max_rpm              (int)     default 5000
//   roller_diameter_m    (double)  default 0.04  (4 cm)
//   gear_ratio           (double)  default 10.0  (motor:roller = 10:1)
//   cmd_refresh_hz       (double)  default 50.0
//   speed_topic          (string)  default "/v_mag_active"
//   speed_scale          (double)  default 0.01  (cm/s -> m/s)
//   max_cmd_cms          (double)  default 300.0 (safety clamp; set <=0 to disable)
//   enable_on_start      (bool)    default true
//
// Optional feedback (polling actual rpm):
//   enable_feedback      (bool)    default false
//   poll_hz              (double)  default 10.0
//   feedback_topic       (string)  default "/kinco/actual_rpm"  (publishes average RPM across motors)

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <stdexcept>
#include <vector>
#include <memory>
#include <mutex>
#include <sstream>
#include <algorithm>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

// ================= Controlword (0x6040) =================
constexpr uint16_t CW_SHUTDOWN         = 0x0006;
constexpr uint16_t CW_SWITCH_ON        = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CW_FAULT_RESET      = 0x0080;
// ========================================================

namespace kinco {

struct MotorConfig {
  int encoder_res = 65536;
  int max_rpm = 5000;

  double roller_diameter_m = 0.04; // 4 cm roller diameter
  double gear_ratio = 10.0;        // motor:roller = 10:1 (motor RPM = roller RPM * 10)

  // Kinco scaling constants (kept as in your code)
  double scale_num = 512.0 * 65536.0; // 512 * encoder_res
  double scale_den = 1875.0;

  void normalize() {
    scale_num = 512.0 * (double)encoder_res;
  }
};

class CanSocketBus {
public:
  explicit CanSocketBus(const std::string& ifname) { open(ifname); }
  ~CanSocketBus() { if (sock_ >= 0) ::close(sock_); }

  CanSocketBus(const CanSocketBus&) = delete;
  CanSocketBus& operator=(const CanSocketBus&) = delete;

  void sendFrame(uint32_t can_id, const uint8_t data[8]) {
    std::lock_guard<std::mutex> lk(mtx_);
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    int n = ::write(sock_, &frame, sizeof(frame));
    if (n != (int)sizeof(frame)) {
      std::ostringstream oss;
      oss << "CAN write failed (n=" << n << " errno=" << errno << ")";
      throw std::runtime_error(oss.str());
    }
  }

  bool recvFrame(struct can_frame* out, int timeout_ms) {
    std::lock_guard<std::mutex> lk(mtx_);
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(sock_, &rfds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = ::select(sock_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) return false;

    int n = ::read(sock_, out, sizeof(*out));
    return n == (int)sizeof(*out);
  }

private:
  int sock_ = -1;
  std::mutex mtx_;

  void open(const std::string& ifname) {
    struct ifreq ifr;
    struct sockaddr_can addr;

    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) throw std::runtime_error("socket(PF_CAN) failed");

    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");

    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) throw std::runtime_error("bind(AF_CAN) failed");

    ROS_INFO("SocketCAN bound to interface: %s", ifname.c_str());
  }
};

class SdoClient {
public:
  SdoClient(CanSocketBus& bus, int node_id) : bus_(bus), node_id_(node_id) {}

  void writeU16(uint16_t index, uint8_t sub, uint16_t val) {
    uint8_t b[2];
    b[0] = (uint8_t)(val & 0xFF);
    b[1] = (uint8_t)((val >> 8) & 0xFF);
    sendSdo(0x2B, index, sub, b, 2);
  }

  void writeI8(uint16_t index, uint8_t sub, int8_t val) {
    uint8_t b[1];
    b[0] = (uint8_t)val;
    sendSdo(0x2F, index, sub, b, 1);
  }

  void writeI32(uint16_t index, uint8_t sub, int32_t val) {
    uint8_t b[4];
    b[0] = (uint8_t)(val & 0xFF);
    b[1] = (uint8_t)((val >> 8) & 0xFF);
    b[2] = (uint8_t)((val >> 16) & 0xFF);
    b[3] = (uint8_t)((val >> 24) & 0xFF);
    sendSdo(0x23, index, sub, b, 4);
  }

  bool readI32(uint16_t index, uint8_t sub, int32_t* out_val, int window_ms) {
    const uint32_t req_cob = 0x600 + (uint32_t)node_id_;
    const uint32_t rep_cob = 0x580 + (uint32_t)node_id_;

    uint8_t req[8];
    std::memset(req, 0, sizeof(req));
    req[0] = 0x40;
    req[1] = (uint8_t)(index & 0xFF);
    req[2] = (uint8_t)((index >> 8) & 0xFF);
    req[3] = sub;
    bus_.sendFrame(req_cob, req);

    const ros::Time end = ros::Time::now() + ros::Duration(window_ms / 1000.0);
    while (ros::Time::now() < end && ros::ok()) {
      struct can_frame f;
      if (!bus_.recvFrame(&f, 5)) continue;

      if ((f.can_id & CAN_EFF_MASK) != rep_cob) continue;
      if (f.can_dlc < 8) continue;
      if (f.data[1] != (uint8_t)(index & 0xFF)) continue;
      if (f.data[2] != (uint8_t)((index >> 8) & 0xFF)) continue;
      if (f.data[3] != sub) continue;

      int32_t v = 0;
      v |= ((int32_t)f.data[4]) << 0;
      v |= ((int32_t)f.data[5]) << 8;
      v |= ((int32_t)f.data[6]) << 16;
      v |= ((int32_t)f.data[7]) << 24;
      *out_val = v;
      return true;
    }
    return false;
  }

  int nodeId() const { return node_id_; }

private:
  CanSocketBus& bus_;
  int node_id_;

  void sendSdo(uint8_t cs, uint16_t index, uint8_t sub, const uint8_t* data_bytes, int data_len) {
    const uint32_t cob_id = 0x600 + (uint32_t)node_id_;
    uint8_t data[8];
    std::memset(data, 0, sizeof(data));
    data[0] = cs;
    data[1] = (uint8_t)(index & 0xFF);
    data[2] = (uint8_t)((index >> 8) & 0xFF);
    data[3] = sub;
    for (int i = 0; i < data_len && i < 4; i++) data[4 + i] = data_bytes[i];
    bus_.sendFrame(cob_id, data);
  }
};

class KincoMotor {
public:
  KincoMotor(CanSocketBus& bus, int node_id, const MotorConfig& cfg)
    : cfg_(cfg), sdo_(bus, node_id)
  {
    cfg_.normalize();
  }

  int nodeId() const { return sdo_.nodeId(); }

  void initProfileVelocityMode() {
    sdo_.writeU16(0x6040, 0x00, CW_SHUTDOWN);
    ros::Duration(0.05).sleep();
    sdo_.writeU16(0x6040, 0x00, CW_SWITCH_ON);
    ros::Duration(0.05).sleep();
    sdo_.writeI8 (0x6060, 0x00, 3);          // Profile Velocity
    ros::Duration(0.05).sleep();
    sdo_.writeI32(0x60FF, 0x00, 0);          // target vel = 0
    ros::Duration(0.05).sleep();
    sdo_.writeU16(0x6040, 0x00, CW_ENABLE_OPERATION);
    ros::Duration(0.05).sleep();

    ROS_INFO("Node %d: Profile Velocity enabled (target=0).", nodeId());
  }

  // Input RPM is MOTOR RPM (after gear ratio)
  void setTargetRpm(double rpm) {
    if (!std::isfinite(rpm)) rpm = 0.0;
    rpm = std::max(- (double)cfg_.max_rpm, std::min((double)cfg_.max_rpm, rpm));

    const int32_t dec = rpmToDriveUnits(rpm);
    sdo_.writeI32(0x60FF, 0x00, dec);
  }

  // Input linear speed is roller linear speed (m/s)
  // This converts to MOTOR rpm by applying gear_ratio, then writes drive units.
  void setTargetLinearMs(double v_ms) { setTargetRpm(msToMotorRpm(v_ms)); }

  bool readActualRpm(double* out_motor_rpm, int window_ms = 20) {
    int32_t dec_val = 0;
    if (!sdo_.readI32(0x606C, 0x00, &dec_val, window_ms)) return false;
    *out_motor_rpm = driveUnitsToRpm(dec_val);
    return true;
  }

  // For your conversion chain: m/s -> roller rpm -> motor rpm
  double msToMotorRpm(double v_ms) const {
    const double circ = M_PI * cfg_.roller_diameter_m;
    if (circ <= 0.0) return 0.0;

    const double roller_rpm = (v_ms / circ) * 60.0;
    return roller_rpm * cfg_.gear_ratio; // <-- your "multiply by 10" happens here
  }

private:
  mutable MotorConfig cfg_;
  SdoClient sdo_;

  int32_t rpmToDriveUnits(double motor_rpm) const {
    const double dec_f = motor_rpm * cfg_.scale_num / cfg_.scale_den;
    return (int32_t)llround(dec_f);
  }

  double driveUnitsToRpm(int32_t dec) const {
    return (double)dec * cfg_.scale_den / cfg_.scale_num;
  }
};

} // namespace kinco

// ----------------- ROS helpers -----------------
static std::vector<int> getNodeIdsParam(ros::NodeHandle& pnh) {
  std::vector<int> ids;
  XmlRpc::XmlRpcValue v;
  if (!pnh.getParam("node_ids", v)) {
    ids.push_back(1);
    return ids;
  }
  if (v.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("~node_ids must be a list, e.g. [1,2,3]");
  }
  for (int i = 0; i < v.size(); i++) {
    if (v[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      throw std::runtime_error("~node_ids must contain ints");
    }
    ids.push_back((int)v[i]);
  }
  if (ids.empty()) ids.push_back(1);
  return ids;
}

static double clampNonNegFinite(double x, double max_val, bool enable_max) {
  if (!std::isfinite(x)) return 0.0;
  if (x < 0.0) x = 0.0;
  if (enable_max && max_val > 0.0 && x > max_val) x = max_val;
  return x;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinco_from_vmag_active");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    // ----- Params -----
    std::string can_iface;
    pnh.param<std::string>("can_iface", can_iface, std::string("can0"));

    std::string speed_topic;
    pnh.param<std::string>("speed_topic", speed_topic, std::string("/v_mag_active"));

    double cmd_refresh_hz = 50.0;
    pnh.param<double>("cmd_refresh_hz", cmd_refresh_hz, cmd_refresh_hz);
    if (cmd_refresh_hz <= 0.0) throw std::runtime_error("~cmd_refresh_hz must be > 0");

    double speed_scale = 0.01; // cm/s -> m/s
    pnh.param<double>("speed_scale", speed_scale, speed_scale);

    double max_cmd_cms = 300.0;
    pnh.param<double>("max_cmd_cms", max_cmd_cms, max_cmd_cms);
    const bool enable_max_cmd = (max_cmd_cms > 0.0);

    bool enable_on_start = true;
    pnh.param<bool>("enable_on_start", enable_on_start, enable_on_start);

    bool enable_feedback = false;
    pnh.param<bool>("enable_feedback", enable_feedback, enable_feedback);
    double poll_hz = 10.0;
    pnh.param<double>("poll_hz", poll_hz, poll_hz);
    std::string feedback_topic;
    pnh.param<std::string>("feedback_topic", feedback_topic, std::string("/kinco/actual_rpm"));

    kinco::MotorConfig cfg;
    pnh.param<int>("encoder_res", cfg.encoder_res, cfg.encoder_res);
    pnh.param<int>("max_rpm", cfg.max_rpm, cfg.max_rpm);
    pnh.param<double>("roller_diameter_m", cfg.roller_diameter_m, cfg.roller_diameter_m); // 0.04
    pnh.param<double>("gear_ratio", cfg.gear_ratio, cfg.gear_ratio);                       // 10.0
    cfg.normalize();

    const auto node_ids = getNodeIdsParam(pnh);

    // ----- CAN bus -----
    kinco::CanSocketBus bus(can_iface);

    // ----- Motors -----
    std::vector<std::unique_ptr<kinco::KincoMotor>> motors;
    motors.reserve(node_ids.size());

    for (int id : node_ids) {
      motors.emplace_back(std::make_unique<kinco::KincoMotor>(bus, id, cfg));
    }

    if (enable_on_start) {
      for (auto& m : motors) {
        ROS_INFO("Initializing node %d...", m->nodeId());
        m->initProfileVelocityMode();
      }
    }

    // ----- Command input (from /v_mag_active) -----
    std::mutex cmd_mtx;
    double latest_v_cms = 0.0;
    bool have_cmd = false;

    ros::Subscriber sub_speed = nh.subscribe<std_msgs::Float64>(
      speed_topic, 50,
      [&](const std_msgs::Float64::ConstPtr& msg){
        std::lock_guard<std::mutex> lk(cmd_mtx);
        latest_v_cms = msg->data; // cm/s
        have_cmd = true;
      }
    );

    // ----- Command timer: apply to all motors -----
    ros::Timer cmd_timer = nh.createTimer(
      ros::Duration(1.0 / cmd_refresh_hz),
      [&](const ros::TimerEvent&){
        double v_cms_local = 0.0;
        bool have = false;
        {
          std::lock_guard<std::mutex> lk(cmd_mtx);
          v_cms_local = latest_v_cms;
          have = have_cmd;
        }
        if (!have) return;

        // sanitize + clamp (cm/s)
        v_cms_local = clampNonNegFinite(v_cms_local, max_cmd_cms, enable_max_cmd);

        // convert to m/s for the roller linear speed
        const double v_ms = v_cms_local * speed_scale; // default 0.01

        // send to each motor
        for (auto& m : motors) {
          m->setTargetLinearMs(v_ms);
        }
      }
    );

    // ----- Optional feedback: publish average motor RPM -----
    ros::Publisher fb_pub;
    ros::Timer fb_timer;

    if (enable_feedback) {
      if (poll_hz <= 0.0) throw std::runtime_error("~poll_hz must be > 0");
      fb_pub = nh.advertise<std_msgs::Float64>(feedback_topic, 10);

      fb_timer = nh.createTimer(
        ros::Duration(1.0 / poll_hz),
        [&](const ros::TimerEvent&){
          if (motors.empty()) return;
          double sum = 0.0;
          int count = 0;
          for (auto& m : motors) {
            double rpm = 0.0;
            if (!m->readActualRpm(&rpm, 20)) continue;
            sum += rpm;
            count++;
          }
          if (count <= 0) return;
          std_msgs::Float64 out;
          out.data = sum / (double)count;
          fb_pub.publish(out);
        }
      );
    }

    ROS_INFO("kinco_from_vmag_active running. Sub: %s, motors: %zu, roller_d=%.3fm, gear=%.2f",
             speed_topic.c_str(), motors.size(), cfg.roller_diameter_m, cfg.gear_ratio);

    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Fatal: %s", e.what());
    return 1;
  }

  return 0;
}
