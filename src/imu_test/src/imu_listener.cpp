#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <string>

// ----------------- helpers -----------------
static inline double deg(double rad) { return rad * 180.0 / M_PI; }

static void quatToRPY_ZYX(double qw, double qx, double qy, double qz, double &roll, double &pitch, double &yaw){
  // roll (x-axis)
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis)
  const double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::fabs(sinp) >= 1.0) pitch = std::copysign(M_PI / 2.0, sinp);
  else                        pitch = std::asin(sinp);

  // yaw (z-axis)
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

static std::string covStr(const boost::array<double, 9>& c){
  // ROS convention: -1 means "unknown"
  // If first element is -1, we’ll treat as unknown.
  if (c[0] < 0.0) return "UNKNOWN";

  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "[% .3g % .3g % .3g; % .3g % .3g % .3g; % .3g % .3g % .3g]",
                c[0], c[1], c[2],
                c[3], c[4], c[5],
                c[6], c[7], c[8]);
  return std::string(buf);
}

// ----------------- state (for optional extra topics) -----------------
static geometry_msgs::Vector3 g_rpy_deg;
static geometry_msgs::Vector3 g_vel_kf;
static bool g_have_rpy = false;
static bool g_have_vel = false;

static void rpyCb(const geometry_msgs::Vector3::ConstPtr& msg){
  g_rpy_deg = *msg;
  g_have_rpy = true;
}

static void velCb(const geometry_msgs::Vector3::ConstPtr& msg){
  g_vel_kf = *msg;
  g_have_vel = true;
}

// ----------------- main IMU callback -----------------
static ros::Time g_last_print(0);

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  // Print throttling
  static double print_hz = 10.0;
  static bool compact = false;

  // Parameters can be changed with rosparam
  ros::param::get("~print_hz", print_hz);
  ros::param::get("~compact", compact);

  const ros::Time now = ros::Time::now();
  const double min_dt = (print_hz > 0.0) ? (1.0 / print_hz) : 0.0;

  if (min_dt > 0.0 && (now - g_last_print).toSec() < min_dt) return;
  g_last_print = now;

  // Extract quaternion
  const double qw = msg->orientation.w;
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;

  // Convert quaternion -> RPY
  double roll = 0, pitch = 0, yaw = 0;
  quatToRPY_ZYX(qw, qx, qy, qz, roll, pitch, yaw);

  // Angular velocity
  const double wx = msg->angular_velocity.x;
  const double wy = msg->angular_velocity.y;
  const double wz = msg->angular_velocity.z;
  const double wmag = std::sqrt(wx*wx + wy*wy + wz*wz);

  // Linear acceleration
  const double ax = msg->linear_acceleration.x;
  const double ay = msg->linear_acceleration.y;
  const double az = msg->linear_acceleration.z;
  const double amag = std::sqrt(ax*ax + ay*ay + az*az);

  // Nice timestamp
  const double t = msg->header.stamp.toSec();

  if (compact) {
    ROS_INFO_STREAM(
      "t=" << std::fixed << std::setprecision(3) << t
      << " | RPY(deg)=[" << std::setprecision(1) << deg(roll) << ", " << deg(pitch) << ", " << deg(yaw) << "]"
      << " | w(rad/s)=[" << std::setprecision(3) << wx << ", " << wy << ", " << wz << "]"
      << " | a(m/s^2)=[" << ax << ", " << ay << ", " << az << "]"
      << (g_have_vel ? (" | v_kf(m/s)=[" + std::to_string(g_vel_kf.x) + ", " + std::to_string(g_vel_kf.y) + ", " + std::to_string(g_vel_kf.z) + "]") : "")
    );
    return;
  }

  // Full pretty print
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss << "\n==================== IMU (sensor_msgs/Imu) ====================\n";
  ss << "stamp:     " << msg->header.stamp << "   (t=" << std::setprecision(3) << t << ")\n";
  ss << "frame_id:  " << msg->header.frame_id << "\n\n";

  ss << "ORIENTATION (quat)\n";
  ss << "  [w=" << std::setprecision(6) << qw
     << ", x=" << qx
     << ", y=" << qy
     << ", z=" << qz << "]\n";
  ss << "  RPY (ZYX): roll=" << roll << " rad (" << deg(roll) << " deg), "
     << "pitch=" << pitch << " rad (" << deg(pitch) << " deg), "
     << "yaw=" << yaw << " rad (" << deg(yaw) << " deg)\n";
  ss << "  orientation_cov: " << covStr(msg->orientation_covariance) << "\n\n";

  ss << "ANGULAR VELOCITY (rad/s)\n";
  ss << "  w = [" << wx << ", " << wy << ", " << wz << "]   |w|=" << wmag << "\n";
  ss << "  angular_vel_cov: " << covStr(msg->angular_velocity_covariance) << "\n\n";

  ss << "LINEAR ACCELERATION (m/s^2)\n";
  ss << "  a = [" << ax << ", " << ay << ", " << az << "]   |a|=" << amag << "\n";
  ss << "  linear_acc_cov:  " << covStr(msg->linear_acceleration_covariance) << "\n\n";

  if (g_have_rpy) {
    ss << "EXTRA TOPIC /imu/rpy (deg)\n";
    ss << "  rpy_deg = [" << g_rpy_deg.x << ", " << g_rpy_deg.y << ", " << g_rpy_deg.z << "]\n\n";
  } else {
    ss << "EXTRA TOPIC /imu/rpy: (not received)\n\n";
  }

  if (g_have_vel) {
    ss << "EXTRA TOPIC /imu/vel_kf (m/s)\n";
    ss << "  v_kf = [" << g_vel_kf.x << ", " << g_vel_kf.y << ", " << g_vel_kf.z << "]\n\n";
  } else {
    ss << "EXTRA TOPIC /imu/vel_kf: (not received)\n\n";
  }

  ss << "===============================================================\n";

  ROS_INFO_STREAM(ss.str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "imu_listener_full");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Params (defaults)
  double print_hz = 10.0;
  bool compact = false;
  pnh.param("print_hz", print_hz, 10.0);
  pnh.param("compact", compact, false);

  ros::Subscriber sub_imu = nh.subscribe("imu", 20, imuCallback);

  // Optional extra topics (safe even if not published)
  ros::Subscriber sub_rpy = nh.subscribe("imu/rpy", 20, rpyCb);
  ros::Subscriber sub_vel = nh.subscribe("imu/vel_kf", 20, velCb);

  ROS_INFO("imu_listener_full started. Subscribed to /imu, /imu/rpy, /imu/vel_kf");
  ROS_INFO("Params: ~print_hz (default 10), ~compact (default false)");

  ros::spin();
  return 0;
}

