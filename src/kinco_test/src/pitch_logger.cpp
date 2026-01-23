// pitch_controller.cpp  (rewritten + actual speed publish + CSV logging)
//
// Subscribes to /v_mag_active (cm/s) and commands Kinco CANopen drives (Profile Velocity mode)
// to match the speed in real-time.
//
// Also publishes feedback (when ~enable_feedback:=true):
//  - /kinco/actual_rpm_avg              (Float64) average motor RPM (0x606C)
//  - /kinco/actual_speed_cms_avg        (Float64) average linear speed equivalent (cm/s)
//  - /kinco/actual_rpm_per_motor        (Float64MultiArray) [rpm_node1, rpm_node2, ...] (same order as ~node_ids)
//  - /kinco/actual_speed_cms_per_motor  (Float64MultiArray) [cms_node1, cms_node2, ...]
//
// Also logs target + actual to CSV (when ~enable_csv_log:=true):
//  Columns:
//    t_sec,target_cms,target_motor_rpm,target_60FF,actual_rpm_avg,actual_cms_avg,
//    actual_rpm_node<ID>..., actual_cms_node<ID>...
//
// Params (~):
//   can_iface            (string)  default "can0"
//   node_ids             (list)    default [1]
//   encoder_res          (int)     default 65536
//   max_rpm              (int)     default 5000
//   roller_diameter_m    (double)  default 0.04
//   gear_ratio           (double)  default 10.0
//   cmd_refresh_hz       (double)  default 50.0
//   speed_topic          (string)  default "/v_mag_active"
//   speed_scale          (double)  default 0.01  (cm/s -> m/s)
//   max_cmd_cms          (double)  default 300.0 (<=0 disables clamp)
//   enable_on_start      (bool)    default true
//
// Feedback:
//   enable_feedback      (bool)    default false
//   poll_hz              (double)  default 10.0
//   feedback_rpm_topic_avg               (string) default "/kinco/actual_rpm_avg"
//   feedback_speed_cms_topic_avg         (string) default "/kinco/actual_speed_cms_avg"
//   feedback_rpm_topic_per_motor         (string) default "/kinco/actual_rpm_per_motor"
//   feedback_speed_cms_topic_per_motor   (string) default "/kinco/actual_speed_cms_per_motor"
//
// CSV logging:
//   enable_csv_log       (bool)    default false
//   csv_dir              (string)  default "/tmp"
//   csv_prefix           (string)  default "kinco_target_actual"
//   csv_log_hz           (double)  default 10.0   (<=0 -> uses poll_hz if feedback enabled else cmd_refresh_hz)

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

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
#include <limits>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <xmlrpcpp/XmlRpcValue.h>
#include <std_msgs/Float64MultiArray.h>

// ================= Controlword (0x6040) =================
constexpr uint16_t CW_SHUTDOWN         = 0x0006;
constexpr uint16_t CW_SWITCH_ON        = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
// ========================================================

constexpr float MAX_VEL = 50.0f; // extra clamp on incoming /v_mag_active (cm/s)

namespace kinco {

struct MotorConfig {
  int encoder_res = 65536;
  int max_rpm = 5000;

  double roller_diameter_m = 0.04;
  double gear_ratio = 10.0;

  // Kinco scaling (common): drive_units = rpm * (512*encoder_res)/1875
  double scale_num = 512.0 * 65536.0;
  double scale_den = 1875.0;

  void normalize() { scale_num = 512.0 * (double)encoder_res; }
};

class CanSocketBus {
public:
  explicit CanSocketBus(const std::string& ifname) { open(ifname); }
  ~CanSocketBus() { if (sock_ >= 0) ::close(sock_); }

  CanSocketBus(const CanSocketBus&) = delete;
  CanSocketBus& operator=(const CanSocketBus&) = delete;

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

  void sendFrame8(uint32_t can_id, const uint8_t data[8]) {
    sendFrameDlc(can_id, data, 8);
  }

  void sendNmt(uint8_t cmd, uint8_t node_id) {
    uint8_t data[2];
    data[0] = cmd;
    data[1] = node_id;
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
  KincoMotor(CanSocketBus& bus, int node_id, const MotorConfig& cfg)
    : cfg_(cfg), sdo_(bus, node_id) {
    cfg_.normalize();
  }

  int nodeId() const { return sdo_.nodeId(); }

  void initProfileVelocityMode() {
    sdo_.writeU16(0x6040, 0x00, CW_SHUTDOWN);
    ros::Duration(0.05).sleep();

    sdo_.writeI8(0x6060, 0x00, 3);  // Profile Velocity
    ros::Duration(0.05).sleep();

    sdo_.writeI32(0x60FF, 0x00, 0); // Target velocity = 0
    ros::Duration(0.05).sleep();

    sdo_.writeU16(0x6040, 0x00, CW_SWITCH_ON);
    ros::Duration(0.05).sleep();

    sdo_.writeU16(0x6040, 0x00, CW_ENABLE_OPERATION);
    ros::Duration(0.05).sleep();

    ROS_INFO("Node %d: Profile Velocity enabled (6060=3).", nodeId());
  }

  // --- command path ---
  void setTargetLinearMs(double roller_v_ms) { setTargetRpm(msToMotorRpm(roller_v_ms)); }

  // --- feedback path ---
  bool readActualMotorRpm(double* out_motor_rpm, int window_ms = 20) {
    int32_t dec_val = 0;
    if (!sdo_.readI32(0x606C, 0x00, &dec_val, window_ms)) return false;
    *out_motor_rpm = driveUnitsToRpm(dec_val);
    return true;
  }

  // --- conversions ---
  double linearMsToMotorRpm(double roller_v_ms) const { return msToMotorRpm(roller_v_ms); }
  int32_t motorRpmToDriveUnits(double motor_rpm) const { return rpmToDriveUnits(motor_rpm); }

  double motorRpmToLinearCms(double motor_rpm) const {
    const double circ = M_PI * cfg_.roller_diameter_m;
    if (circ <= 0.0) return 0.0;
    const double roller_rpm = motor_rpm / cfg_.gear_ratio;
    const double v_ms = (roller_rpm / 60.0) * circ;
    return v_ms * 100.0;
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
    return roller_rpm * cfg_.gear_ratio;
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

// ---------------- CSV logger ----------------
struct CsvLogger {
  std::ofstream ofs;
  bool header_written = false;
};

static std::string makeTimestampedCsvName(const std::string& dir,
                                          const std::string& prefix)
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);

  std::ostringstream oss;
  oss << dir << "/" << prefix << "_"
      << std::setfill('0')
      << std::setw(4) << (tm.tm_year + 1900) << "-"
      << std::setw(2) << (tm.tm_mon + 1) << "-"
      << std::setw(2) << tm.tm_mday << "_"
      << std::setw(2) << tm.tm_hour << "-"
      << std::setw(2) << tm.tm_min << "-"
      << std::setw(2) << tm.tm_sec
      << ".csv";
  return oss.str();
}

static void initCsvLogger(CsvLogger& L,
                          const std::vector<int>& node_ids,
                          const std::string& csv_path)
{
  L.ofs.open(csv_path, std::ios::out | std::ios::trunc);
  if (!L.ofs.is_open()) throw std::runtime_error("Failed to open CSV: " + csv_path);

    L.ofs << "t_sec"
      << ",target_cms"
      << ",target_motor_rpm"
      << ",target_60FF"
      << ",actual_rpm_avg"
      << ",actual_cms_avg"
      << ",foot_contact_pair_2";


  for (int id : node_ids) L.ofs << ",actual_rpm_node" << id;
  for (int id : node_ids) L.ofs << ",actual_cms_node" << id;

  L.ofs << "\n";
  L.ofs.flush();
  L.header_written = true;
}

static void logCsvRow(CsvLogger& L,
                      double t_sec,
                      double target_cms,
                      double target_motor_rpm,
                      int32_t target_60FF,
                      double actual_rpm_avg,
                      double actual_cms_avg,
                      double foot_contact_pair_2,
                      const std::vector<double>& actual_rpm_per,
                      const std::vector<double>& actual_cms_per,
                      const std::vector<int>& node_ids)
{
  if (!L.ofs.is_open() || !L.header_written) return;

  const double nan = std::numeric_limits<double>::quiet_NaN();

  L.ofs << std::fixed << std::setprecision(6)
        << t_sec << ","
        << target_cms << ","
        << target_motor_rpm << ","
        << target_60FF << ","
        << actual_rpm_avg << ","
        << actual_cms_avg << ","
        << foot_contact_pair_2;


  for (size_t i = 0; i < node_ids.size(); i++) {
    const double v = (i < actual_rpm_per.size()) ? actual_rpm_per[i] : nan;
    L.ofs << "," << v;
  }
  for (size_t i = 0; i < node_ids.size(); i++) {
    const double v = (i < actual_cms_per.size()) ? actual_cms_per[i] : nan;
    L.ofs << "," << v;
  }

  L.ofs << "\n";
  L.ofs.flush(); // safe if Ctrl-C
}
// --------------------------------------------------------

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

    // feedback params
    bool enable_feedback = true;
    pnh.param<bool>("enable_feedback", enable_feedback, enable_feedback);

    double poll_hz = 10.0;
    pnh.param<double>("poll_hz", poll_hz, poll_hz);

    std::string feedback_rpm_topic_avg, feedback_speed_cms_topic_avg;
    std::string feedback_rpm_topic_per_motor, feedback_speed_cms_topic_per_motor;

    pnh.param<std::string>("feedback_rpm_topic_avg",
                           feedback_rpm_topic_avg,
                           std::string("/kinco/actual_rpm_avg"));
    pnh.param<std::string>("feedback_speed_cms_topic_avg",
                           feedback_speed_cms_topic_avg,
                           std::string("/kinco/actual_speed_cms_avg"));
    pnh.param<std::string>("feedback_rpm_topic_per_motor",
                           feedback_rpm_topic_per_motor,
                           std::string("/kinco/actual_rpm_per_motor"));
    pnh.param<std::string>("feedback_speed_cms_topic_per_motor",
                           feedback_speed_cms_topic_per_motor,
                           std::string("/kinco/actual_speed_cms_per_motor"));

    // csv params
    bool enable_csv_log = true;
    pnh.param<bool>("enable_csv_log", enable_csv_log, enable_csv_log);

    std::string csv_dir, csv_prefix;
    pnh.param<std::string>("csv_dir", csv_dir, std::string("/tmp"));
    pnh.param<std::string>("csv_prefix", csv_prefix, std::string("kinco_target_actual"));

    double csv_log_hz = 0.0;
    pnh.param<double>("csv_log_hz", csv_log_hz, csv_log_hz);

    kinco::MotorConfig cfg;
    pnh.param<int>("encoder_res", cfg.encoder_res, cfg.encoder_res);
    pnh.param<int>("max_rpm", cfg.max_rpm, cfg.max_rpm);
    pnh.param<double>("roller_diameter_m", cfg.roller_diameter_m, cfg.roller_diameter_m);
    pnh.param<double>("gear_ratio", cfg.gear_ratio, cfg.gear_ratio);
    cfg.normalize();

    const auto node_ids = getNodeIdsParam(pnh);

    kinco::CanSocketBus bus(can_iface);

    // Start nodes
    for (int id : node_ids) {
      bus.sendNmt(0x01, (uint8_t)id);
      ros::Duration(0.01).sleep();
    }
    ros::Duration(0.05).sleep();

    std::vector<std::unique_ptr<kinco::KincoMotor>> motors;
    motors.reserve(node_ids.size());
    for (int id : node_ids) motors.emplace_back(std::make_unique<kinco::KincoMotor>(bus, id, cfg));

    if (enable_on_start) {
      for (auto& m : motors) {
        ROS_INFO("Initializing node %d...", m->nodeId());
        m->initProfileVelocityMode();
      }
    }

    // ---- command state ----
    std::mutex cmd_mtx;
    double latest_v_cms = 0.0;
    bool have_cmd = false;

    // ---- foot contact state (from /foot_contact_pair) ----
    std::mutex contact_mtx;
    double latest_contact2 = std::numeric_limits<double>::quiet_NaN();
    bool have_contact2 = false;


    // Keep latest "target" values for CSV (set by cmd_timer).
    double last_target_cms = 0.0;
    double last_target_motor_rpm = 0.0;
    int32_t last_target_60FF = 0;
    ros::Time last_target_stamp = ros::Time(0);

    ros::Subscriber sub_speed = nh.subscribe<std_msgs::Float64>(
      speed_topic, 50,
      [&](const std_msgs::Float64::ConstPtr& msg){
        float data = static_cast<float>(msg->data) / 3.0f; // keep your /3 behavior

        if (!std::isfinite(data)) data = 0.0f;
        if (data < 0.0f) data = 0.0f;
        if (data > MAX_VEL) data = MAX_VEL;

        std::lock_guard<std::mutex> lk(cmd_mtx);
        latest_v_cms = static_cast<double>(data);
        have_cmd = true;
      }
    );
    ros::Subscriber sub_contact = nh.subscribe<std_msgs::Float64MultiArray>(
        "/foot_contact_pair", 50,
        [&](const std_msgs::Float64MultiArray::ConstPtr& msg){
            if (msg->data.size() < 3) return;   // need data[2]
            std::lock_guard<std::mutex> lk(contact_mtx);
            latest_contact2 = msg->data[3];
            have_contact2 = true;
        }
    );


    // ---- feedback publishers ----
    ros::Publisher fb_rpm_avg_pub;
    ros::Publisher fb_speed_cms_avg_pub;
    ros::Publisher fb_rpm_per_pub;
    ros::Publisher fb_speed_cms_per_pub;

    if (enable_feedback) {
      if (poll_hz <= 0.0) throw std::runtime_error("~poll_hz must be > 0");
      fb_rpm_avg_pub = nh.advertise<std_msgs::Float64>(feedback_rpm_topic_avg, 10);
      fb_speed_cms_avg_pub = nh.advertise<std_msgs::Float64>(feedback_speed_cms_topic_avg, 10);
      fb_rpm_per_pub = nh.advertise<std_msgs::Float64MultiArray>(feedback_rpm_topic_per_motor, 10);
      fb_speed_cms_per_pub = nh.advertise<std_msgs::Float64MultiArray>(feedback_speed_cms_topic_per_motor, 10);
    }

    // ---- CSV init ----
    CsvLogger csv;
    std::string csv_path;
    if (enable_csv_log) {
      csv_path = makeTimestampedCsvName(csv_dir, csv_prefix);
      initCsvLogger(csv, node_ids, csv_path);
      ROS_INFO("CSV logging enabled: %s", csv_path.c_str());
    }

    // Decide CSV log rate
    double effective_csv_hz = csv_log_hz;
    if (effective_csv_hz <= 0.0) {
      if (enable_feedback) effective_csv_hz = poll_hz;
      else effective_csv_hz = cmd_refresh_hz;
    }
    if (enable_csv_log && effective_csv_hz <= 0.0) {
      throw std::runtime_error("CSV logging enabled but effective log hz <= 0");
    }

    // ---- command timer ----
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

        // For logging/debug, compute "target" numbers from motor 0 (same for all if same cfg)
        double target_motor_rpm = 0.0;
        int32_t target_60FF = 0;
        if (!motors.empty()) {
          target_motor_rpm = motors[0]->linearMsToMotorRpm(roller_v_ms);
          target_60FF = motors[0]->motorRpmToDriveUnits(target_motor_rpm);
        }

        // Send to all motors
        for (auto& m : motors) {
          const double motor_rpm = m->linearMsToMotorRpm(roller_v_ms);
          const int32_t units60FF = m->motorRpmToDriveUnits(motor_rpm);

          ROS_INFO_THROTTLE(0.5,
            "CAN cmd node=%d: v=%.2f cm/s (%.3f m/s), motor=%.1f rpm, 0x60FF=%d",
            m->nodeId(), v_cms_local, roller_v_ms, motor_rpm, units60FF
          );

          m->setTargetLinearMs(roller_v_ms);
        }

        // Store latest target for CSV
        {
          std::lock_guard<std::mutex> lk(cmd_mtx);
          last_target_cms = v_cms_local;
          last_target_motor_rpm = target_motor_rpm;
          last_target_60FF = target_60FF;
          last_target_stamp = ros::Time::now();
        }
      }
    );

    // ---- feedback timer (and/or CSV logging) ----
    ros::Timer fb_timer;
    ros::Timer csv_timer;

    if (enable_feedback) {
      fb_timer = nh.createTimer(
        ros::Duration(1.0 / poll_hz),
        [&](const ros::TimerEvent&){
          if (motors.empty()) return;

          std_msgs::Float64MultiArray rpm_arr;
          std_msgs::Float64MultiArray cms_arr;
          rpm_arr.data.reserve(motors.size());
          cms_arr.data.reserve(motors.size());

          std::vector<double> rpm_vec;
          std::vector<double> cms_vec;
          rpm_vec.reserve(motors.size());
          cms_vec.reserve(motors.size());

          double sum_rpm = 0.0;
          double sum_cms = 0.0;
          int count = 0;

          const double nan = std::numeric_limits<double>::quiet_NaN();

          for (auto& m : motors) {
            double rpm = 0.0;
            if (!m->readActualMotorRpm(&rpm, 20)) {
              rpm_arr.data.push_back(nan);
              cms_arr.data.push_back(nan);
              rpm_vec.push_back(nan);
              cms_vec.push_back(nan);
              continue;
            }

            const double cms = m->motorRpmToLinearCms(rpm);

            rpm_arr.data.push_back(rpm);
            cms_arr.data.push_back(cms);
            rpm_vec.push_back(rpm);
            cms_vec.push_back(cms);

            sum_rpm += rpm;
            sum_cms += cms;
            count++;
          }

          // publish per-motor arrays
          fb_rpm_per_pub.publish(rpm_arr);
          fb_speed_cms_per_pub.publish(cms_arr);

          // publish averages
          double avg_rpm = nan;
          double avg_cms = nan;
          if (count > 0) {
            avg_rpm = sum_rpm / (double)count;
            avg_cms = sum_cms / (double)count;

            std_msgs::Float64 out_rpm, out_cms;
            out_rpm.data = avg_rpm;
            out_cms.data = avg_cms;
            fb_rpm_avg_pub.publish(out_rpm);
            fb_speed_cms_avg_pub.publish(out_cms);
          }

          // CSV logging (synchronized to feedback timer if enabled)
          if (enable_csv_log && (effective_csv_hz == poll_hz || csv_log_hz <= 0.0)) {
            double tgt_cms, tgt_rpm;
            int32_t tgt_60FF;
            {
              std::lock_guard<std::mutex> lk(cmd_mtx);
              tgt_cms  = last_target_cms;
              tgt_rpm  = last_target_motor_rpm;
              tgt_60FF = last_target_60FF;
            }
            double contact2_snapshot = std::numeric_limits<double>::quiet_NaN();
            {
            std::lock_guard<std::mutex> lk(contact_mtx);
            if (have_contact2) contact2_snapshot = latest_contact2;
            }


            logCsvRow(csv,
                ros::Time::now().toSec(),
                tgt_cms,
                tgt_rpm,
                tgt_60FF,
                avg_rpm,
                avg_cms,
                contact2_snapshot,
                rpm_vec,
                cms_vec,
                node_ids);
          }
        }
      );
    }

    // If CSV enabled but we are NOT logging from feedback timer (either feedback disabled
    // or user set csv_log_hz different), create a dedicated CSV timer.
    if (enable_csv_log) {
      const bool logging_from_feedback_timer =
        enable_feedback && (effective_csv_hz == poll_hz || csv_log_hz <= 0.0);

      if (!logging_from_feedback_timer) {
        csv_timer = nh.createTimer(
          ros::Duration(1.0 / effective_csv_hz),
          [&](const ros::TimerEvent&){
            // target snapshot
            double tgt_cms, tgt_rpm;
            int32_t tgt_60FF;
            {
              std::lock_guard<std::mutex> lk(cmd_mtx);
              tgt_cms  = last_target_cms;
              tgt_rpm  = last_target_motor_rpm;
              tgt_60FF = last_target_60FF;
            }

            // if feedback disabled, actual fields are NaN
            const double nan = std::numeric_limits<double>::quiet_NaN();
            double contact2_snapshot = std::numeric_limits<double>::quiet_NaN();
            {
            std::lock_guard<std::mutex> lk(contact_mtx);
            if (have_contact2) contact2_snapshot = latest_contact2;
            }

            std::vector<double> empty;
            logCsvRow(csv,
                    ros::Time::now().toSec(),
                    tgt_cms,
                    tgt_rpm,
                    tgt_60FF,
                    nan,
                    nan,
                    contact2_snapshot,
                    empty,
                    empty,
                    node_ids);
          }
        );
      }
    }

    ROS_INFO("kinco_from_vmag_active running. Sub: %s, motors: %zu, roller_d=%.3fm, gear=%.2f",
             speed_topic.c_str(), motors.size(), cfg.roller_diameter_m, cfg.gear_ratio);

    if (enable_feedback) {
      ROS_INFO("Feedback enabled @ %.1f Hz. Publishes RPM + linear speed (cm/s).", poll_hz);
    }
    if (enable_csv_log) {
      ROS_INFO("CSV enabled @ %.1f Hz. File: %s", effective_csv_hz, csv_path.c_str());
    }

    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Fatal: %s", e.what());
    return 1;
  }

  return 0;
}
