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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace kinco {

struct MotorConfig {
  int encoder_res = 65536;
  int max_rpm = 5000;
  double roller_diameter_m = 0.10;

  // These scalars are from your existing conversion.
  // Keep as-is unless your drive scaling differs.
  double scale_num = 512.0 * 65536.0;  // 512 * encoder_res
  double scale_den = 1875.0;          // denominator
};

class CanSocketBus {
public:
  explicit CanSocketBus(const std::string& ifname) { open(ifname); }
  ~CanSocketBus() {
    if (sock_ >= 0) ::close(sock_);
  }

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

    // Request upload
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
  KincoMotor(CanSocketBus& bus, int node_id, MotorConfig cfg)
      : cfg_(cfg), sdo_(bus, node_id) {
    cfg_.scale_num = 512.0 * (double)cfg_.encoder_res;  // keep consistent
  }

  int nodeId() const { return sdo_.nodeId(); }

  void initProfileVelocityMode() {
    // 6040=0006 (Shutdown)
    // 6040=0007 (Switch on)
    // 6060=3    (Profile Velocity)
    // 60FF=0    (Target velocity = 0)
    // 6040=000F (Enable operation)
    sdo_.writeU16(0x6040, 0x00, 0x0006);
    ros::Duration(0.05).sleep();
    sdo_.writeU16(0x6040, 0x00, 0x0007);
    ros::Duration(0.05).sleep();
    sdo_.writeI8(0x6060, 0x00, 3);
    ros::Duration(0.05).sleep();
    sdo_.writeI32(0x60FF, 0x00, 0);
    ros::Duration(0.05).sleep();
    sdo_.writeU16(0x6040, 0x00, 0x000F);
    ros::Duration(0.05).sleep();

    ROS_INFO("Node %d: Profile Velocity enabled (target=0).", nodeId());
  }

  void setTargetRpm(double rpm) {
    if (rpm > cfg_.max_rpm) rpm = cfg_.max_rpm;
    if (rpm < -cfg_.max_rpm) rpm = -cfg_.max_rpm;

    const int32_t dec = rpmToDriveUnits(rpm);
    sdo_.writeI32(0x60FF, 0x00, dec);
  }

  void setTargetLinearMs(double v_ms) { setTargetRpm(msToRpm(v_ms)); }

  bool readActualRpm(double* out_rpm, int window_ms = 20) {
    int32_t dec_val = 0;
    if (!sdo_.readI32(0x606C, 0x00, &dec_val, window_ms)) return false;
    *out_rpm = driveUnitsToRpm(dec_val);
    return true;
  }

  double msToRpm(double v_ms) const {
    const double circ = M_PI * cfg_.roller_diameter_m;
    if (circ <= 0.0) return 0.0;
    return (v_ms / circ) * 60.0;
  }

private:
  MotorConfig cfg_;
  SdoClient sdo_;

  int32_t rpmToDriveUnits(double rpm) const {
    // Your original: dec = rpm * 512 * ENCODER_RES / 1875
    const double dec_f = rpm * cfg_.scale_num / cfg_.scale_den;
    return (int32_t)llround(dec_f);
  }

  double driveUnitsToRpm(int32_t dec) const {
    // Your original: rpm = dec * 1875 / (512 * ENCODER_RES)
    return (double)dec * cfg_.scale_den / cfg_.scale_num;
  }
};

}  // namespace kinco

// ---- Outside-the-class helpers (what you asked for) ----
static void update_motor_speed_ms(kinco::KincoMotor& m, double v_ms) { m.setTargetLinearMs(v_ms); }
static void update_motor_speed_rpm(kinco::KincoMotor& m, double rpm) { m.setTargetRpm(rpm); }

// ---- Example ROS node wiring for multiple motors ----
struct MotorCtx {
  std::unique_ptr<kinco::KincoMotor> motor;
  ros::Subscriber sub_cmd_ms;
  ros::Publisher pub_actual_rpm;

  double latest_cmd_ms = 0.0;
  bool have_cmd = false;
};

static std::vector<int> getNodeIdsParam(ros::NodeHandle& pnh) {
  std::vector<int> ids;
  XmlRpc::XmlRpcValue v;
  if (!pnh.getParam("node_ids", v)) {
    ids.push_back(1);
    return ids;
  }
  if (v.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::runtime_error("~node_ids must be a list");

  for (int i = 0; i < v.size(); i++) {
    if (v[i].getType() != XmlRpc::XmlRpcValue::TypeInt) throw std::runtime_error("~node_ids must contain ints");
    ids.push_back((int)v[i]);
  }
  if (ids.empty()) ids.push_back(1);
  return ids;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinco_multi_canopen_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    std::string can_iface;
    pnh.param<std::string>("can_iface", can_iface, std::string("can0"));

    kinco::MotorConfig cfg;
    pnh.param<int>("encoder_res", cfg.encoder_res, 65536);
    pnh.param<int>("max_rpm", cfg.max_rpm, 5000);
    pnh.param<double>("roller_diameter_m", cfg.roller_diameter_m, 0.10);
    cfg.scale_num = 512.0 * (double)cfg.encoder_res;

    double cmd_refresh_hz = 50.0;
    double poll_hz = 20.0;
    pnh.param<double>("cmd_refresh_hz", cmd_refresh_hz, 50.0);
    pnh.param<double>("poll_hz", poll_hz, 20.0);
    if (cmd_refresh_hz <= 0.0) throw std::runtime_error("~cmd_refresh_hz must be > 0");
    if (poll_hz <= 0.0) throw std::runtime_error("~poll_hz must be > 0");

    const auto node_ids = getNodeIdsParam(pnh);

    kinco::CanSocketBus bus(can_iface);

    std::vector<MotorCtx> motors;
    motors.reserve(node_ids.size());

    // Create + init motors OUTSIDE the motor class (here in main)
    for (int id : node_ids) {
      MotorCtx ctx;
      ctx.motor = std::make_unique<kinco::KincoMotor>(bus, id, cfg);

      ROS_INFO("Initializing node %d...", id);
      ctx.motor->initProfileVelocityMode();

      // Topics: motor_<id>/cmd_vel_ms, motor_<id>/actual_velocity_rpm
      std::ostringstream cmd_topic, fb_topic;
      cmd_topic << "motor_" << id << "/cmd_vel_ms";
      fb_topic << "motor_" << id << "/actual_velocity_rpm";

      ctx.pub_actual_rpm = nh.advertise<std_msgs::Float64>(fb_topic.str(), 10);

      // Update command OUTSIDE class: just store the latest cmd, timer applies it
      ctx.sub_cmd_ms = nh.subscribe<std_msgs::Float64>(
          cmd_topic.str(), 10,
          [&ctx](const std_msgs::Float64::ConstPtr& msg) {
            ctx.latest_cmd_ms = msg->data;
            ctx.have_cmd = true;
          });

      motors.push_back(std::move(ctx));
    }

    ros::Timer cmd_timer = nh.createTimer(
        ros::Duration(1.0 / cmd_refresh_hz),
        [&motors](const ros::TimerEvent&) {
          for (auto& ctx : motors) {
            if (!ctx.have_cmd) continue;
            // Outside function controlling speed:
            update_motor_speed_ms(*ctx.motor, ctx.latest_cmd_ms);
          }
        });

    ros::Timer poll_timer = nh.createTimer(
        ros::Duration(1.0 / poll_hz),
        [&motors](const ros::TimerEvent&) {
          for (auto& ctx : motors) {
            double rpm = 0.0;
            if (!ctx.motor->readActualRpm(&rpm, 20)) continue;
            std_msgs::Float64 out;
            out.data = rpm;
            ctx.pub_actual_rpm.publish(out);
          }
        });

    ROS_INFO("kinco_multi_canopen_node started with %zu motor(s).", motors.size());
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Fatal: %s", e.what());
    return 1;
  }

  return 0;
}
