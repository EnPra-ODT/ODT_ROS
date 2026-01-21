// imu_controller.cpp
//
// Dual-channel IMU velocity estimator + deltaRPY publisher per foot, plus an "active" fused velocity.
//
// Subscribes (per-channel):
//   - /imu_data_left, /imu_data_right   (std_msgs/Float64MultiArray), expected size==7
//     Layout expected:
//       [0]=t_ms, [1]=ax, [2]=ay, [3]=az, [4]=roll_deg, [5]=pitch_deg, [6]=yaw_deg
//
// Publishes:
//   - /v_mag_left,  /v_mag_right        (std_msgs/Float64)           [cm/s]
//   - /deltaRPY_left, /deltaRPY_right   (std_msgs/Float64MultiArray) [dR,dP,dY,sumAbs] in deg
//   - /v_mag_active                     (std_msgs/Float64)           (smoothed "active" velocity)
//
// Params (~):
//   active_out_topic      (string)  default "/v_mag_active"   <-- IMPORTANT (no joystick needed)
//   active_stale_sec      (double)  default 0.20
//   move_thresh_deg       (double)  default 1.0
//   alpha_up              (double)  default 0.7
//   alpha_down            (double)  default 0.05
//   max_drop_cms_per_s    (double)  default 200.0   (0 disables)
//   fuse_mode             (int)     default 0       (0=MAX(vL,vR), 1=deltaRPY-weighted blend)
//
// Per-channel params under ~left/* and ~right/*:
//   topic (string)           default /imu_data_left|right
//   out_topic (string)       default /v_mag_left|right
//   out_topic_RPY (string)   default /deltaRPY_left|right
//   deadband_x/y/z (double)  defaults 0.15/0.15/0.30
//   zupt_steps (int)         default 5
//   Q,R,lpf_alpha,q_v,q_b,r_zupt
//   dt_min, dt_max
//   rpy_zupt_thresh_deg
//
// Notes:
// - This node requires t_ms to increase (data[0]). If it stays constant, dt<=0 and no output.
// - If your IMU layout differs (e.g., yaw at data[4]), update the indices accordingly.

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <boost/bind.hpp>
#include <cmath>
#include <string>
#include <algorithm>

// ----------------- Per-axis state -----------------
struct AxisState {
  double a_hat = 0.0;
  double P_1d  = 1.0;

  bool   has_light_prev = false;
  double a_light_prev   = 0.0;

  double v2 = 0.0;
  double b2 = 0.0;

  double P00 = 1.0, P01 = 0.0, P10 = 0.0, P11 = 1.0;
};

// ----------------- Helpers -----------------
static inline float wrapDiffDeg(float cur, float prev){
  float d = cur - prev;
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

struct DeltaRPY {
  float dR  = 0.f;
  float dP  = 0.f;
  float dY  = 0.f;
  float sum = 0.f;
};

static inline DeltaRPY calcDeltaRPY(const float cur[3], float prev[3]){
  DeltaRPY out;
  out.dR  = wrapDiffDeg(cur[0], prev[0]);
  out.dP  = wrapDiffDeg(cur[1], prev[1]);
  out.dY  = wrapDiffDeg(cur[2], prev[2]);
  out.sum = std::fabs(out.dR) + std::fabs(out.dP) + std::fabs(out.dY);

  prev[0] = cur[0];
  prev[1] = cur[1];
  prev[2] = cur[2];
  return out;
}

static inline bool finiteOrZero(double &x){
  if (!std::isfinite(x)) { x = 0.0; return false; }
  return true;
}

// ----------------- Node -----------------
class ImuController {
public:
  explicit ImuController(ros::NodeHandle& pnh) : pnh_(pnh) {
    ros::NodeHandle lnh(pnh_, "left");
    ros::NodeHandle rnh(pnh_, "right");

    loadChannelParams(lnh, left_,  "left");
    loadChannelParams(rnh, right_, "right");

    // Active-selection params (node-level)
    pnh_.param("active_stale_sec",   active_stale_sec_,   0.20);
    pnh_.param("move_thresh_deg",    move_thresh_deg_,    1.0);
    pnh_.param("alpha_up",           alpha_up_,           0.7);
    pnh_.param("alpha_down",         alpha_down_,         0.05);
    pnh_.param("max_drop_cms_per_s", max_drop_cms_per_s_, 200.0);
    pnh_.param("fuse_mode",          fuse_mode_,          0);

    std::string active_out_topic;
    pnh_.param<std::string>("active_out_topic", active_out_topic, std::string("/v_mag_active"));
    active_vmag_pub_ = pnh_.advertise<std_msgs::Float64>(active_out_topic, 10);

    left_.sub = pnh_.subscribe<std_msgs::Float64MultiArray>(
      left_.topic, 50, boost::bind(&ImuController::cb, this, _1, &left_)
    );
    right_.sub = pnh_.subscribe<std_msgs::Float64MultiArray>(
      right_.topic, 50, boost::bind(&ImuController::cb, this, _1, &right_)
    );

    watchdog_ = pnh_.createTimer(ros::Duration(1.0), &ImuController::watchdogCb, this);

    ROS_INFO_STREAM("[imu_controller] Left  topic: " << left_.topic
                    << " -> v: " << left_.out_topic_vmag
                    << " rpy: " << left_.out_topic_rpy);
    ROS_INFO_STREAM("[imu_controller] Right topic: " << right_.topic
                    << " -> v: " << right_.out_topic_vmag
                    << " rpy: " << right_.out_topic_rpy);
    ROS_INFO_STREAM("[imu_controller] Active velocity topic: " << active_out_topic);
  }

private:
  struct Channel {
    std::string label;

    std::string topic;
    std::string out_topic_vmag;
    std::string out_topic_rpy;

    ros::Subscriber sub;
    ros::Publisher  vmag_pub;
    ros::Publisher  rpy_pub;

    std_msgs::Float64 vmag_msg;
    std_msgs::Float64MultiArray rpy_msg;

    ros::Time last_msg_time;

    // Filter params (per channel)
    double deadband_x = 0.15, deadband_y = 0.15, deadband_z = 0.30;
    int    zupt_steps = 5;

    double Q = 0.05, R = 0.20;
    double lpf_alpha = 0.3;
    double q_v = 0.5;
    double q_b = 0.01;
    double r_zupt = 1e-4;

    double dt_min = 0.001;
    double dt_max = 0.10;
    double rpy_zupt_thresh_deg = 5.0;

    // Time tracking
    bool   have_prev_time = false;
    double t_prev_ms = 0.0;

    // RPY tracking
    bool  have_prev_rpy = false;
    float prev_rpy[3] = {0.f, 0.f, 0.f};
    int   rpy_still_count = 0;

    // Axis states
    AxisState x, y, z;

    // Cached outputs (for active selection)
    bool      has_latest   = false;
    ros::Time latest_stamp;
    double    vmag         = 0.0;  // cm/s
    double    delta_rpy    = 0.0;  // deg-sum (sumAbs)
  };

  void loadChannelParams(ros::NodeHandle& nhc, Channel& C, const std::string& label){
    C.label = label;

    nhc.param<std::string>("topic",        C.topic,          std::string("/imu_data_" + label));
    nhc.param<std::string>("out_topic",    C.out_topic_vmag, std::string("/v_mag_" + label));
    nhc.param<std::string>("out_topic_RPY",C.out_topic_rpy,  std::string("/deltaRPY_" + label));

    nhc.param("deadband_x", C.deadband_x, 0.15);
    nhc.param("deadband_y", C.deadband_y, 0.15);
    nhc.param("deadband_z", C.deadband_z, 0.30);
    nhc.param("zupt_steps", C.zupt_steps, 5);

    nhc.param("Q", C.Q, 0.05);
    nhc.param("R", C.R, 0.20);
    nhc.param("lpf_alpha", C.lpf_alpha, 0.3);

    nhc.param("q_v", C.q_v, 0.5);
    nhc.param("q_b", C.q_b, 0.01);
    nhc.param("r_zupt", C.r_zupt, 1e-4);

    nhc.param("dt_min", C.dt_min, 0.001);
    nhc.param("dt_max", C.dt_max, 0.10);

    nhc.param("rpy_zupt_thresh_deg", C.rpy_zupt_thresh_deg, 5.0);

    // Advertise output topics (use pnh_ so they're global absolute names you pass in)
    C.vmag_pub = pnh_.advertise<std_msgs::Float64>(C.out_topic_vmag, 10);
    C.rpy_pub  = pnh_.advertise<std_msgs::Float64MultiArray>(C.out_topic_rpy, 10);

    C.last_msg_time = ros::Time(0);
  }

  void watchdogCb(const ros::TimerEvent&){
    warnChannel(left_);
    warnChannel(right_);
  }

  void warnChannel(const Channel& C){
    if (C.last_msg_time.isZero()) {
      ROS_WARN_THROTTLE(2.0, "[%s] No IMU messages received yet.", C.label.c_str());
      return;
    }
    if ((ros::Time::now() - C.last_msg_time).toSec() > 2.0) {
      ROS_WARN_THROTTLE(2.0, "[%s] No IMU messages received in >2s.", C.label.c_str());
    }
  }

  // ---------- Active selection ----------
  void publishActive(double dt_fallback){
    double dt_active = dt_fallback;

    const ros::Time now = ros::Time::now();
    if (!have_active_time_) {
      last_active_pub_time_ = now;
      have_active_time_ = true;
    } else {
      dt_active = (now - last_active_pub_time_).toSec();
      last_active_pub_time_ = now;
      if (dt_active <= 1e-6) dt_active = dt_fallback;
    }

    auto isFresh = [&](const Channel& C){
      if (!C.has_latest) return false;
      return (now - C.latest_stamp).toSec() <= active_stale_sec_;
    };

    const bool Lfresh = isFresh(left_);
    const bool Rfresh = isFresh(right_);

    // If neither fresh: decay toward 0
    if (!Lfresh && !Rfresh) {
      const double v_meas = 0.0;
      const double a = (v_meas > v_active_out_) ? alpha_up_ : alpha_down_;
      v_active_out_ += a * (v_meas - v_active_out_);
      active_vmag_msg_.data = v_active_out_;
      active_vmag_pub_.publish(active_vmag_msg_);
      return;
    }

    const double vL = Lfresh ? left_.vmag  : 0.0;
    const double vR = Rfresh ? right_.vmag : 0.0;

    const double dL = Lfresh ? left_.delta_rpy  : 0.0;
    const double dR = Rfresh ? right_.delta_rpy : 0.0;

    const bool Lmoving = (dL >= move_thresh_deg_);
    const bool Rmoving = (dR >= move_thresh_deg_);

    double v_meas = 0.0;

    if (Lmoving || Rmoving) {
      if (fuse_mode_ == 0) {
        v_meas = std::max(vL, vR);
      } else {
        const double wL_raw = std::max(0.0, dL - move_thresh_deg_);
        const double wR_raw = std::max(0.0, dR - move_thresh_deg_);
        const double sum = wL_raw + wR_raw + 1e-9;
        const double wL = wL_raw / sum;
        v_meas = wL * vL + (1.0 - wL) * vR;
      }
    } else {
      v_meas = 0.0;
    }

    // Envelope follower
    const double a = (v_meas > v_active_out_) ? alpha_up_ : alpha_down_;
    double v_next = v_active_out_ + a * (v_meas - v_active_out_);

    // Optional: clamp maximum drop rate
    if (max_drop_cms_per_s_ > 0.0 && dt_active > 1e-6) {
      const double max_drop = max_drop_cms_per_s_ * dt_active;
      if (v_next < v_active_out_ - max_drop) v_next = v_active_out_ - max_drop;
    }

    v_active_out_ = v_next;
    active_vmag_msg_.data = v_active_out_;
    active_vmag_pub_.publish(active_vmag_msg_);
  }

  // ---------- Filter axis ----------
  void processAxis(
    AxisState& S,
    double a_in,
    double deadband,
    double F00, double F01, double F10, double F11,
    double dt,
    bool still_global,
    const Channel& C
  ){
    // deadband on measurement
    double z = a_in;
    if (std::abs(z) < deadband) z = 0.0;

    // 1D KF on accel
    S.P_1d += C.Q;
    const double K = S.P_1d / (S.P_1d + C.R);
    S.a_hat += K * (z - S.a_hat);
    S.P_1d  *= (1.0 - K);

    // LPF accel (used for integration)
    double a_light = a_in;
    if (!S.has_light_prev) {
      S.has_light_prev = true;
    } else {
      a_light = C.lpf_alpha * S.a_light_prev + (1.0 - C.lpf_alpha) * a_in;
    }
    S.a_light_prev = a_light;

    const double u = a_light;

    // integrate with bias estimate
    S.v2 += (u - S.b2) * dt;

    // 2-state covariance predict
    const double A00 = F00 * S.P00 + F01 * S.P10;
    const double A01 = F00 * S.P01 + F01 * S.P11;
    const double A10 = F10 * S.P00 + F11 * S.P10;
    const double A11 = F10 * S.P01 + F11 * S.P11;

    const double P00p = A00 * F00 + A01 * F01 + C.q_v * dt * dt;
    const double P01p = A00 * F10 + A01 * F11;
    const double P10p = A10 * F00 + A11 * F01;
    const double P11p = A10 * F10 + A11 * F11 + C.q_b * dt;

    S.P00 = P00p; S.P01 = P01p; S.P10 = P10p; S.P11 = P11p;

    // ZUPT update when still
    if (still_global) {
      const double innov = -S.v2;
      const double Szz   = S.P00 + C.r_zupt;

      const double K0 = S.P00 / Szz;
      const double K1 = S.P10 / Szz;

      S.v2 += K0 * innov;
      S.b2 += K1 * innov;

      const double P00_old = S.P00;
      const double P01_old = S.P01;
      const double P10_old = S.P10;
      const double P11_old = S.P11;

      S.P00 = (1.0 - K0) * P00_old;
      S.P01 = (1.0 - K0) * P01_old;
      S.P10 = P10_old - K1 * P00_old;
      S.P11 = P11_old - K1 * P01_old;
    }
  }

  // ---------- Callback ----------
  void cb(const std_msgs::Float64MultiArray::ConstPtr& msg, Channel* C){
    C->last_msg_time = ros::Time::now();

    const auto& arr = msg->data;
    if (arr.size() != 7) {
      ROS_WARN_THROTTLE(1.0, "[%s] expected size==7, got %zu", C->label.c_str(), arr.size());
      return;
    }

    double t_ms = arr[0];
    double ax   = arr[1];
    double ay   = arr[2];
    double az   = arr[3];

    finiteOrZero(t_ms);
    finiteOrZero(ax);
    finiteOrZero(ay);
    finiteOrZero(az);

    // IMPORTANT: layout is [roll,pitch,yaw] at indices 4..6
    const float cur_rpy[3] = {
      static_cast<float>(arr[4]), // roll_deg
      static_cast<float>(arr[5]), // pitch_deg
      static_cast<float>(arr[6])  // yaw_deg
    };

    // init on first message
    if (!C->have_prev_time) {
      C->t_prev_ms = t_ms;
      C->have_prev_time = true;

      C->prev_rpy[0] = cur_rpy[0];
      C->prev_rpy[1] = cur_rpy[1];
      C->prev_rpy[2] = cur_rpy[2];
      C->have_prev_rpy = true;
      return;
    }

    const double dt = (t_ms - C->t_prev_ms) * 1e-3;
    if (!(dt > 0.0)) {
      ROS_WARN_THROTTLE(1.0, "[%s] Non-positive dt. Resyncing.", C->label.c_str());
      C->t_prev_ms = t_ms;
      return;
    }
    C->t_prev_ms = t_ms;

    if (dt < C->dt_min || dt > C->dt_max) {
      ROS_WARN_THROTTLE(1.0, "[%s] dt=%.6f rejected (min=%.6f max=%.6f).",
                        C->label.c_str(), dt, C->dt_min, C->dt_max);
      return;
    }

    const double F00 = 1.0, F01 = -dt;
    const double F10 = 0.0, F11 =  1.0;

    // deltaRPY (deg)
    if (!C->have_prev_rpy) {
      C->prev_rpy[0] = cur_rpy[0];
      C->prev_rpy[1] = cur_rpy[1];
      C->prev_rpy[2] = cur_rpy[2];
      C->have_prev_rpy = true;
      return;
    }

    DeltaRPY drpy = calcDeltaRPY(cur_rpy, C->prev_rpy);

    if (drpy.sum <= static_cast<float>(C->rpy_zupt_thresh_deg)) C->rpy_still_count++;
    else C->rpy_still_count = 0;

    const bool still_global = (C->rpy_still_count >= C->zupt_steps);

    // Filter/integrate each axis
    processAxis(C->x, ax, C->deadband_x, F00, F01, F10, F11, dt, still_global, *C);
    processAxis(C->y, ay, C->deadband_y, F00, F01, F10, F11, dt, still_global, *C);
    processAxis(C->z, az, C->deadband_z, F00, F01, F10, F11, dt, still_global, *C);

    const double vx = C->x.v2;
    const double vy = C->y.v2;
    const double vz = C->z.v2;

    const double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz) * 100.0; // cm/s

    // publish vmag
    C->vmag_msg.data = v_mag;
    C->vmag_pub.publish(C->vmag_msg);

    // publish deltaRPY as MultiArray: [dR,dP,dY,sum]
    C->rpy_msg.data.resize(4);
    C->rpy_msg.data[0] = drpy.dR;
    C->rpy_msg.data[1] = drpy.dP;
    C->rpy_msg.data[2] = drpy.dY;
    C->rpy_msg.data[3] = drpy.sum;
    C->rpy_pub.publish(C->rpy_msg);

    // cache latest for active selection
    C->vmag         = v_mag;
    C->delta_rpy    = drpy.sum;
    C->latest_stamp = ros::Time::now();
    C->has_latest   = true;

    publishActive(dt);
  }

private:
  ros::NodeHandle pnh_;
  ros::Timer watchdog_;

  Channel left_;
  Channel right_;

  // Active-selection state (node-level)
  ros::Publisher active_vmag_pub_;
  std_msgs::Float64 active_vmag_msg_;
  double v_active_out_ = 0.0;

  // Params (tune in launch)
  double active_stale_sec_   = 0.20;
  double move_thresh_deg_    = 1.0;
  double alpha_up_           = 0.7;
  double alpha_down_         = 0.05;
  double max_drop_cms_per_s_ = 200.0;
  int    fuse_mode_          = 0;

  ros::Time last_active_pub_time_;
  bool have_active_time_ = false;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "imu_controller");
  ros::NodeHandle pnh("~");

  ImuController node(pnh);
  ros::spin();
  return 0;
}
