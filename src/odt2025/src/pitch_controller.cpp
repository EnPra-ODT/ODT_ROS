// kinco_speed_from_vmag_active.cpp
//
// Subscribes to /v_mag_active (cm/s) and commands Kinco CANopen drives (Profile Velocity mode)
// to match the speed in real-time.
//
// Converts: cm/s -> m/s -> roller RPM (roller dia) -> motor RPM (gear ratio) -> drive units -> 0x60FF
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
// Optional feedback:
//   enable_feedback      (bool)    default false
//   poll_hz              (double)  default 10.0
//   feedback_topic       (string)  default "/kinco/actual_rpm"

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

#include <xmlrpcpp/XmlRpcValue.h>

// ================= Controlword (0x6040) =================
constexpr uint16_t CW_SHUTDOWN         = 0x0006;
constexpr uint16_t CW_SWITCH_ON        = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
// ========================================================

constexpr float MAX_VEL = 50.0f; // max velocity


namespace kinco {

struct MotorConfig {
  int encoder_res = 65536;
  int max_rpm = 5000;

  double roller_diameter_m = 0.042; // 4 cm roller diameter
  double gear_ratio = 10.0;        // motor:roller = 10:1

  // Kinco scaling constants (as in your original code)
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

  // Generic send with custom DLC
  void sendFrameDlc(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    std::lock_guard<std::mutex> lk(mtx_);
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id  = can_id;
    frame.can_dlc = dlc;
    if (dlc > 0) std::memcpy(frame.data, data, dlc);

    int n = ::write(sock_, &frame, sizeof(frame));
    if (n != (int)sizeof(frame)) {
      std::ostringstream oss;
      oss << "CAN write failed (n=" << n << " errno=" << errno << ")";
      throw std::runtime_error(oss.str());
    }
  }

  // SDO always DLC=8
  void sendFrame8(uint32_t can_id, const uint8_t data[8]) {
    sendFrameDlc(can_id, data, 8);
  }

  // NMT: COB-ID 0x000, DLC=2
  void sendNmt(uint8_t cmd, uint8_t node_id) {
    uint8_t data[2];
    data[0] = cmd;     // 0x01=start, 0x02=stop, 0x80=pre-op, 0x81=reset node
    data[1] = node_id; // 0 = all nodes or specific node
    sendFrameDlc(0x000, data, 2);
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
    bus_.sendFrame8(req_cob, req);

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
    bus_.sendFrame8(cob_id, data);
  }
};

class KincoMotor {
public:
  // For logging
  double linearMsToMotorRpm(double roller_v_ms) const { return msToMotorRpm(roller_v_ms); }
  int32_t motorRpmToDriveUnits(double motor_rpm) const { return rpmToDriveUnits(motor_rpm); }

  KincoMotor(CanSocketBus& bus, int node_id, const MotorConfig& cfg)
    : cfg_(cfg), sdo_(bus, node_id)
  {
    cfg_.normalize();
  }

  int nodeId() const { return sdo_.nodeId(); }

  void initProfileVelocityMode() {
    // Shutdown first
    sdo_.writeU16(0x6040, 0x00, CW_SHUTDOWN);
    ros::Duration(0.05).sleep();

    // Set mode (Profile Velocity = 3)
    sdo_.writeI8(0x6060, 0x00, 3);
    ros::Duration(0.05).sleep();

    // Target velocity = 0
    sdo_.writeI32(0x60FF, 0x00, 0);
    ros::Duration(0.05).sleep();

    // Switch on + enable op
    sdo_.writeU16(0x6040, 0x00, CW_SWITCH_ON);
    ros::Duration(0.05).sleep();

    sdo_.writeU16(0x6040, 0x00, CW_ENABLE_OPERATION);
    ros::Duration(0.05).sleep();

    ROS_INFO("Node %d: Profile Velocity enabled (6060=3).", nodeId());
  }

  void setTargetLinearMs(double roller_v_ms) { setTargetRpm(msToMotorRpm(roller_v_ms)); }

  bool readActualRpm(double* out_motor_rpm, int window_ms = 20) {
    int32_t dec_val = 0;
    if (!sdo_.readI32(0x606C, 0x00, &dec_val, window_ms)) return false;
    *out_motor_rpm = driveUnitsToRpm(dec_val);
    return true;
  }

private:
  mutable MotorConfig cfg_;
  SdoClient sdo_;

  void setTargetRpm(double motor_rpm) {
    if (!std::isfinite(motor_rpm)) motor_rpm = 0.0;
    motor_rpm = std::max(-(double)cfg_.max_rpm, std::min((double)cfg_.max_rpm, motor_rpm));
    const int32_t dec = rpmToDriveUnits(motor_rpm);
    sdo_.writeI32(0x60FF, 0x00, dec);
  }

  double msToMotorRpm(double roller_v_ms) const {
    const double circ = M_PI * cfg_.roller_diameter_m;
    if (circ <= 0.0) return 0.0;
    const double roller_rpm = (roller_v_ms / circ) * 60.0;
    return roller_rpm * cfg_.gear_ratio; // <-- your multiply-by-10
  }

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
  if (v.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error("~node_ids must be a list, e.g. [1,2,3]");

  for (int i = 0; i < v.size(); i++) {
    if (v[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
      throw std::runtime_error("~node_ids must contain ints");
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
    pnh.param<double>("roller_diameter_m", cfg.roller_diameter_m, cfg.roller_diameter_m);
    pnh.param<double>("gear_ratio", cfg.gear_ratio, cfg.gear_ratio);
    cfg.normalize();

    const auto node_ids = getNodeIdsParam(pnh);

    // ----- CAN bus -----
    kinco::CanSocketBus bus(can_iface);

    // Put all nodes into OPERATIONAL
    for (int id : node_ids) {
      bus.sendNmt(0x01, (uint8_t)id);
      ros::Duration(0.01).sleep();
    }
    ros::Duration(0.05).sleep();

    // ----- Motors -----
    std::vector<std::unique_ptr<kinco::KincoMotor>> motors;
    motors.reserve(node_ids.size());
    for (int id : node_ids) motors.emplace_back(std::make_unique<kinco::KincoMotor>(bus, id, cfg));

    if (enable_on_start) {
      for (auto& m : motors) {
        ROS_INFO("Initializing node %d...", m->nodeId());
        m->initProfileVelocityMode();
      }
    }

    // ----- Speed input -----
    std::mutex cmd_mtx;
    double latest_v_cms = 0.0;
    bool have_cmd = false;

    ros::Subscriber sub_speed = nh.subscribe<std_msgs::Float64>(
      speed_topic, 50,
      [&](const std_msgs::Float64::ConstPtr& msg){
        float data = static_cast<float>(msg->data);

        if (!std::isfinite(data)) data = 0.0f;
        if (data < 0.0f) data = 0.0f;          // optional but recommended
        if (data > MAX_VEL) data = MAX_VEL;    // your clamp

        std::lock_guard<std::mutex> lk(cmd_mtx);
        latest_v_cms = static_cast<double>(data);
        have_cmd = true;
      }
    );


    // ----- Command timer -----
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

        v_cms_local = clampNonNegFinite(v_cms_local, max_cmd_cms, enable_max_cmd);
        const double roller_v_ms = v_cms_local * speed_scale;

        for (auto& m : motors) {
        // compute what will be sent
        const double motor_rpm = m->linearMsToMotorRpm(roller_v_ms);
        const int32_t units60FF = m->motorRpmToDriveUnits(motor_rpm);

        ROS_INFO_THROTTLE(0.5,
          "CAN cmd node=%d: v=%.2f cm/s (%.3f m/s), motor=%.1f rpm, 0x60FF=%d",
          m->nodeId(), v_cms_local, roller_v_ms, motor_rpm, units60FF
        );

        m->setTargetLinearMs(roller_v_ms); // this does the SDO write to 0x60FF
      }

      }
    );

    // ----- Optional feedback -----
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
