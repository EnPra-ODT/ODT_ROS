// absolute_yaw_calc.cpp
//
// Publishes a "persistent absolute yaw command" using only IMU yaw + deltaRPY gating.
// This is a test node (no Dynamixel, no load cells).
//
// Concept:
//  - Subscribe: /imu_data_left, /imu_data_right (std_msgs/Float64MultiArray) -> uses data[4] as yaw deg
//  - Subscribe: /deltaRPY_left, /deltaRPY_right (std_msgs/Float64MultiArray) -> uses data[2] as deltaYaw deg (adjust if needed)
//  - Determine swing/stance per foot using deltaYaw thresholds with hysteresis + consecutive counts
//  - Option A behavior: when a foot enters swing, capture yaw0 and cmd_base; while swinging, update cmd_yaw = cmd_base + (yaw - yaw0) wrapped
//  - If both swing, choose active foot by larger |deltaYaw|
//  - Publish: /cmd_yaw_deg (std_msgs/Float64MultiArray) as [cmd_yaw_deg, activeFoot, leftSwing, rightSwing, dyawL, dyawR, yawL, yawR]
//
// Params (~private):
//  left_topic (string)         default "/imu_data_left"
//  right_topic (string)        default "/imu_data_right"
//  delta_left_topic (string)   default "/deltaRPY_left"
//  delta_right_topic (string)  default "/deltaRPY_right"
//  out_topic (string)          default "/cmd_yaw_deg"
//
//  yaw_index (int)             default 4   (index in imu_data_* array)
//  dyaw_index (int)            default 2   (index in deltaRPY_* array)
//  use_norm (bool)             default false (if true, motion metric = norm(dR,dP,dY) instead of |dY|)
//
//  move_th_deg (double)        default 2.0
//  still_th_deg (double)       default 1.0
//  move_N (int)                default 3
//  still_N (int)               default 5
//
//  pub_rate_hz (double)        default 50.0  (timer publish rate)
//  zero_on_start (bool)        default true
//
// Build (catkin):
//  add_executable(absolute_yaw_calc src/absolute_yaw_calc.cpp)
//  target_link_libraries(absolute_yaw_calc ${catkin_LIBRARIES})
//
#include <cmath>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

static inline double wrapDeg(double a){
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}
static inline double wrapDiffDeg(double a_minus_b){
  return wrapDeg(a_minus_b);
}

enum class ActiveFoot : int {
  NONE  = 0,
  LEFT  = 1,
  RIGHT = 2
};

struct FootPhase {
  bool   swing = false;
  int    move_cnt = 0;
  int    still_cnt = 0;
  double yaw0_deg = 0.0;
  double cmd_base_deg = 0.0;
};

struct FootImu {
  bool have_yaw = false;
  bool have_drpy = false;

  double yaw_deg = 0.0;

  // deltaRPY raw
  double droll = 0.0;
  double dpitch = 0.0;
  double dyaw = 0.0;
};

static FootImu   g_L, g_R;
static FootPhase g_Lp, g_Rp;

static double g_cmd_yaw_deg = 0.0;
static bool   g_zero_on_start = true;
static bool   g_have_zero = false;
static double g_zero_deg = 0.0;

static ActiveFoot g_active = ActiveFoot::NONE;

// params
static int    g_yaw_index  = 4;
static int    g_dyaw_index = 2;
static bool   g_use_norm   = false;

static double g_move_th_deg  = 2.0;
static double g_still_th_deg = 1.0;
static int    g_move_N       = 3;
static int    g_still_N      = 5;

static ros::Publisher g_pub;

static double motionMetricDeg(const FootImu &f){
  if (!g_use_norm) return std::fabs(f.dyaw);
  const double n = std::sqrt(f.droll*f.droll + f.dpitch*f.dpitch + f.dyaw*f.dyaw);
  return n;
}

static void updateFootPhase(FootPhase &fp, const FootImu &f){
  if (!f.have_yaw || !f.have_drpy) return;

  const double m = motionMetricDeg(f);

  if (!fp.swing) {
    if (m >= g_move_th_deg) fp.move_cnt++;
    else fp.move_cnt = 0;

    if (fp.move_cnt >= g_move_N) {
      fp.swing = true;
      fp.move_cnt = 0;
      fp.still_cnt = 0;

      fp.yaw0_deg = f.yaw_deg;
      fp.cmd_base_deg = g_cmd_yaw_deg;
    }
  } else {
    if (m <= g_still_th_deg) fp.still_cnt++;
    else fp.still_cnt = 0;

    if (fp.still_cnt >= g_still_N) {
      fp.swing = false;
      fp.still_cnt = 0;
      fp.move_cnt = 0;
      // Keep g_cmd_yaw_deg frozen when swing ends
    }
  }
}

static ActiveFoot chooseActiveFoot(){
  const bool Ls = g_Lp.swing;
  const bool Rs = g_Rp.swing;

  if (Ls && Rs) {
    const double ml = motionMetricDeg(g_L);
    const double mr = motionMetricDeg(g_R);
    return (ml >= mr) ? ActiveFoot::LEFT : ActiveFoot::RIGHT;
  }
  if (Ls) return ActiveFoot::LEFT;
  if (Rs) return ActiveFoot::RIGHT;
  return ActiveFoot::NONE;
}

static void computeCmdYaw(){
  if (!g_L.have_yaw || !g_R.have_yaw || !g_L.have_drpy || !g_R.have_drpy) return;

  // Update swing/stance state from motion
  updateFootPhase(g_Lp, g_L);
  updateFootPhase(g_Rp, g_R);

  g_active = chooseActiveFoot();

  // Update persistent cmd yaw only while a foot is swinging (Option A)
  if (g_active == ActiveFoot::LEFT) {
    const double d = wrapDiffDeg(g_L.yaw_deg - g_Lp.yaw0_deg);
    g_cmd_yaw_deg = g_Lp.cmd_base_deg + d;
  } else if (g_active == ActiveFoot::RIGHT) {
    const double d = wrapDiffDeg(g_R.yaw_deg - g_Rp.yaw0_deg);
    g_cmd_yaw_deg = g_Rp.cmd_base_deg + d;
  }

  if (g_zero_on_start && !g_have_zero) {
    g_zero_deg = g_cmd_yaw_deg;
    g_have_zero = true;
    // do not early return; publish zeroed output immediately
  }
}

static void publishOut(const ros::TimerEvent&){
  computeCmdYaw();

  // output (optionally zeroed)
  const double out_yaw = (g_zero_on_start && g_have_zero) ? (g_cmd_yaw_deg - g_zero_deg) : g_cmd_yaw_deg;

  std_msgs::Float64MultiArray out;
  out.data.resize(8);
  out.data[0] = out_yaw;
  out.data[1] = static_cast<double>(static_cast<int>(g_active));
  out.data[2] = g_Lp.swing ? 1.0 : 0.0;
  out.data[3] = g_Rp.swing ? 1.0 : 0.0;
  out.data[4] = g_L.dyaw;
  out.data[5] = g_R.dyaw;
  out.data[6] = g_L.yaw_deg;
  out.data[7] = g_R.yaw_deg;

  g_pub.publish(out);

  ROS_INFO_THROTTLE(0.5,
    "cmd=%.2f act=%d swing[L,R]=[%d,%d] dyaw[L,R]=[%.2f,%.2f] yaw[L,R]=[%.2f,%.2f]",
    out_yaw,
    (int)g_active,
    (int)g_Lp.swing, (int)g_Rp.swing,
    g_L.dyaw, g_R.dyaw,
    g_L.yaw_deg, g_R.yaw_deg
  );
}

static void imuLeftCb(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if ((int)msg->data.size() <= g_yaw_index) {
    ROS_WARN_THROTTLE(1.0, "imu_data_left: need index %d, got size %zu", g_yaw_index, msg->data.size());
    return;
  }
  g_L.yaw_deg = msg->data[g_yaw_index];
  g_L.have_yaw = true;
}

static void imuRightCb(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if ((int)msg->data.size() <= g_yaw_index) {
    ROS_WARN_THROTTLE(1.0, "imu_data_right: need index %d, got size %zu", g_yaw_index, msg->data.size());
    return;
  }
  g_R.yaw_deg = msg->data[g_yaw_index];
  g_R.have_yaw = true;
}

static void deltaLeftCb(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if (msg->data.size() < 3) {
    ROS_WARN_THROTTLE(1.0, "deltaRPY_left: expected >=3, got %zu", msg->data.size());
    return;
  }
  // store full for optional norm
  g_L.droll  = msg->data[0];
  g_L.dpitch = msg->data[1];

  if ((int)msg->data.size() <= g_dyaw_index) {
    ROS_WARN_THROTTLE(1.0, "deltaRPY_left: need index %d, got size %zu", g_dyaw_index, msg->data.size());
    return;
  }
  g_L.dyaw = msg->data[g_dyaw_index];
  g_L.have_drpy = true;
}

static void deltaRightCb(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if (msg->data.size() < 3) {
    ROS_WARN_THROTTLE(1.0, "deltaRPY_right: expected >=3, got %zu", msg->data.size());
    return;
  }
  g_R.droll  = msg->data[0];
  g_R.dpitch = msg->data[1];

  if ((int)msg->data.size() <= g_dyaw_index) {
    ROS_WARN_THROTTLE(1.0, "deltaRPY_right: need index %d, got size %zu", g_dyaw_index, msg->data.size());
    return;
  }
  g_R.dyaw = msg->data[g_dyaw_index];
  g_R.have_drpy = true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "absolute_yaw_calc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string left_topic       = "/imu_data_left";
  std::string right_topic      = "/imu_data_right";
  std::string delta_left_topic = "/deltaRPY_left";
  std::string delta_right_topic= "/deltaRPY_right";
  std::string out_topic        = "/cmd_yaw_deg";

  pnh.param<std::string>("left_topic", left_topic, left_topic);
  pnh.param<std::string>("right_topic", right_topic, right_topic);
  pnh.param<std::string>("delta_left_topic", delta_left_topic, delta_left_topic);
  pnh.param<std::string>("delta_right_topic", delta_right_topic, delta_right_topic);
  pnh.param<std::string>("out_topic", out_topic, out_topic);

  pnh.param<int>("yaw_index", g_yaw_index, 4);
  pnh.param<int>("dyaw_index", g_dyaw_index, 2);
  pnh.param<bool>("use_norm", g_use_norm, false);

  pnh.param<double>("move_th_deg", g_move_th_deg, 2.0);
  pnh.param<double>("still_th_deg", g_still_th_deg, 1.0);
  pnh.param<int>("move_N", g_move_N, 3);
  pnh.param<int>("still_N", g_still_N, 5);

  pnh.param<bool>("zero_on_start", g_zero_on_start, true);

  double pub_rate_hz = 50.0;
  pnh.param<double>("pub_rate_hz", pub_rate_hz, 50.0);
  pub_rate_hz = std::max(1.0, pub_rate_hz);

  ros::Subscriber subL  = nh.subscribe(left_topic,        50, imuLeftCb);
  ros::Subscriber subR  = nh.subscribe(right_topic,       50, imuRightCb);
  ros::Subscriber subDL = nh.subscribe(delta_left_topic,  50, deltaLeftCb);
  ros::Subscriber subDR = nh.subscribe(delta_right_topic, 50, deltaRightCb);

  g_pub = nh.advertise<std_msgs::Float64MultiArray>(out_topic, 10);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0 / pub_rate_hz), publishOut);

  ROS_INFO("absolute_yaw_calc running.");
  ROS_INFO("IMU L: %s  IMU R: %s", left_topic.c_str(), right_topic.c_str());
  ROS_INFO("dRPY L: %s  dRPY R: %s", delta_left_topic.c_str(), delta_right_topic.c_str());
  ROS_INFO("Out: %s", out_topic.c_str());
  ROS_INFO("yaw_index=%d dyaw_index=%d use_norm=%d move_th=%.2f still_th=%.2f move_N=%d still_N=%d pub_rate=%.1f zero_on_start=%d",
           g_yaw_index, g_dyaw_index, (int)g_use_norm,
           g_move_th_deg, g_still_th_deg, g_move_N, g_still_N,
           pub_rate_hz, (int)g_zero_on_start);

  ros::spin();
  return 0;
}
