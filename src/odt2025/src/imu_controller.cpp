// velocity_calculator_dual.cpp
//
// Dual-channel IMU velocity estimator with deltaRPY motion detection.
//
// OVERVIEW:
// - Processes two IMU streams (left/right feet) independently
// - Estimates velocity using Kalman filtering with ZUPT (Zero velocity UPdaTe)
// - Detects motion using RPY (Roll/Pitch/Yaw) changes
// - Fuses both channels into a single "active" velocity output
//
// PUBLISHED TOPICS:
//   /v_mag_left, /v_mag_right          - Per-foot velocity magnitude (cm/s)
//   /deltaRPY_left, /deltaRPY_right    - Per-foot angular changes [dR, dP, dY, sum] (deg)
//   /v_mag_active                       - Fused active velocity (cm/s)
//
// SUBSCRIBED TOPICS:
//   /imu_data_left, /imu_data_right    - IMU data arrays [t_ms, ax, ay, az, roll, pitch, yaw]

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <boost/bind.hpp>
#include <cmath>
#include <string>
#include <algorithm>

// ============================================================================
// CONSTANTS & CONFIGURATION
// ============================================================================

namespace defaults {
  // Global fusion parameters
  const double ACTIVE_STALE_SEC = 0.20;      // Max age for "fresh" data
  const double MOVE_THRESH_DEG = 1.0;        // Angular motion threshold
  const double ALPHA_UP = 0.7;               // Attack time for velocity rise
  const double ALPHA_DOWN = 0.05;            // Decay time for velocity fall
  const double MAX_DROP_CMS_PER_S = 200.0;   // Max velocity decrease rate
  const int FUSE_MODE = 0;                   // 0=MAX, 1=weighted blend
  
  // Per-channel filter parameters
  const double DEADBAND_X = 0.15;            // Accel deadband (m/s²)
  const double DEADBAND_Y = 0.15;
  const double DEADBAND_Z = 0.30;
  const int ZUPT_STEPS = 5;                  // Stillness confirmation count
  
  const double Q = 0.05;                     // Process noise (1D filter)
  const double R = 0.20;                     // Measurement noise (1D filter)
  const double LPF_ALPHA = 0.3;              // Low-pass filter coefficient
  
  const double Q_V = 0.5;                    // Velocity process noise (2D filter)
  const double Q_B = 0.01;                   // Bias process noise (2D filter)
  const double R_ZUPT = 1e-4;                // ZUPT measurement noise
  
  const double DT_MIN = 0.001;               // Min valid timestep (s)
  const double DT_MAX = 0.10;                // Max valid timestep (s)
  const double RPY_ZUPT_THRESH_DEG = 5.0;    // Angular stillness threshold
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Single-axis Kalman filter state (velocity + bias estimation)
struct AxisState {
  // 1D smoothing filter for acceleration
  double accel_filtered = 0.0;
  double accel_variance = 1.0;
  
  // Light low-pass filter for integration
  bool has_prev_accel = false;
  double prev_accel_lpf = 0.0;
  
  // 2D state: velocity and bias
  double velocity = 0.0;
  double bias = 0.0;
  
  // 2x2 covariance matrix
  double P00 = 1.0, P01 = 0.0;
  double P10 = 0.0, P11 = 1.0;
};

// Angular change measurement (Roll/Pitch/Yaw)
struct DeltaRPY {
  float delta_roll = 0.0f;
  float delta_pitch = 0.0f;
  float delta_yaw = 0.0f;
  float sum_absolute = 0.0f;  // Total angular motion
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Wrap angle difference to [-180, 180] degrees
inline float wrapAngleDifference(float current, float previous) {
  float diff = current - previous;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

// Calculate change in Roll/Pitch/Yaw and update previous values
inline DeltaRPY calculateDeltaRPY(const float current[3], float previous[3]) {
  DeltaRPY result;
  
  result.delta_roll = wrapAngleDifference(current[0], previous[0]);
  result.delta_pitch = wrapAngleDifference(current[1], previous[1]);
  result.delta_yaw = wrapAngleDifference(current[2], previous[2]);
  
  result.sum_absolute = std::fabs(result.delta_roll) + std::fabs(result.delta_pitch) + std::fabs(result.delta_yaw);
  
  // Update previous values
  previous[0] = current[0];
  previous[1] = current[1];
  previous[2] = current[2];
  
  return result;
}

// ============================================================================
// MAIN NODE CLASS
// ============================================================================

class DualChannelVelocityEstimator {
public:
  explicit DualChannelVelocityEstimator(ros::NodeHandle& private_nh);

private:
  // Per-channel configuration and state
  struct Channel {
    std::string name;  // "left" or "right"
    
    // Topic names
    std::string input_topic;
    std::string velocity_topic;
    std::string delta_rpy_topic;
    
    // ROS communication
    ros::Subscriber subscriber;
    ros::Publisher velocity_publisher;
    ros::Publisher delta_rpy_publisher;
    ros::Time last_message_time;
    
    // Filter parameters (tunable per channel)
    double deadband_x, deadband_y, deadband_z;
    int zupt_confirmation_steps;
    double q_1d, r_1d, lpf_alpha;
    double q_velocity, q_bias, r_zupt;
    double dt_min, dt_max;
    double rpy_stillness_threshold_deg;
    
    // Time tracking
    bool has_previous_timestamp = false;
    double previous_time_ms = 0.0;
    
    // RPY motion detection
    bool has_previous_rpy = false;
    float previous_rpy[3] = {0.0f, 0.0f, 0.0f};
    int stillness_counter = 0;
    
    // Per-axis filter states
    AxisState x_axis, y_axis, z_axis;
    
    // Latest outputs (for fusion)
    bool has_valid_output = false;
    ros::Time output_timestamp;
    double velocity_magnitude = 0.0;  // cm/s
    double delta_rpy_sum = 0.0;       // degrees
    
    // Message buffers
    std_msgs::Float64 velocity_msg;
    std_msgs::Float64MultiArray delta_rpy_msg;
  };
  
  // Initialize channel parameters from ROS parameter server
  void loadChannelParameters(ros::NodeHandle& channel_nh, Channel& channel, 
                            const std::string& name);
  
  // Process incoming IMU data for a channel
  void imuCallback(const std_msgs::Float64MultiArray::ConstPtr& msg, Channel* channel);
  
  // Apply Kalman filter to single axis
  void filterAxis(AxisState& state, double accel_measured, double deadband,
                 double state_transition_00, double state_transition_01,
                 double state_transition_10, double state_transition_11,
                 double dt, bool apply_zupt, const Channel& channel);
  
  // Fuse left/right channels into active velocity
  void publishActiveVelocity(double fallback_dt);
  
  // Check for stale data and warn
  void watchdogCallback(const ros::TimerEvent& event);
  void checkChannelHealth(const Channel& channel);
  
  // Member variables
  ros::NodeHandle node_handle_;
  ros::Timer watchdog_timer_;
  
  Channel left_channel_;
  Channel right_channel_;
  
  // Active velocity fusion
  ros::Publisher active_velocity_publisher_;
  std_msgs::Float64 active_velocity_msg_;
  double active_velocity_output_ = 0.0;
  
  // Fusion parameters
  double data_freshness_threshold_sec_;
  double motion_threshold_deg_;
  double alpha_rising_;
  double alpha_falling_;
  double max_velocity_drop_rate_;
  int fusion_mode_;
  
  // Timing for active velocity updates
  ros::Time last_active_publish_time_;
  bool has_active_publish_time_ = false;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

DualChannelVelocityEstimator::DualChannelVelocityEstimator(ros::NodeHandle& private_nh)
    : node_handle_(private_nh) {
  
  // Create namespaced node handles for each channel
  ros::NodeHandle left_nh(private_nh, "left");
  ros::NodeHandle right_nh(private_nh, "right");
  
  loadChannelParameters(left_nh, left_channel_, "left");
  loadChannelParameters(right_nh, right_channel_, "right");
  
  // Load global fusion parameters
  private_nh.param("active_stale_sec", data_freshness_threshold_sec_, 
                   defaults::ACTIVE_STALE_SEC);
  private_nh.param("move_thresh_deg", motion_threshold_deg_, 
                   defaults::MOVE_THRESH_DEG);
  private_nh.param("alpha_up", alpha_rising_, defaults::ALPHA_UP);
  private_nh.param("alpha_down", alpha_falling_, defaults::ALPHA_DOWN);
  private_nh.param("max_drop_cms_per_s", max_velocity_drop_rate_, 
                   defaults::MAX_DROP_CMS_PER_S);
  private_nh.param("fuse_mode", fusion_mode_, defaults::FUSE_MODE);
  
  // Set up publishers and subscribers
  active_velocity_publisher_ = private_nh.advertise<std_msgs::Float64>(
      "/v_mag_active", 10);
  
  left_channel_.subscriber = private_nh.subscribe<std_msgs::Float64MultiArray>(
      left_channel_.input_topic, 50,
      boost::bind(&DualChannelVelocityEstimator::imuCallback, this, _1, &left_channel_));
  
  right_channel_.subscriber = private_nh.subscribe<std_msgs::Float64MultiArray>(
      right_channel_.input_topic, 50,
      boost::bind(&DualChannelVelocityEstimator::imuCallback, this, _1, &right_channel_));
  
  watchdog_timer_ = private_nh.createTimer(
      ros::Duration(1.0), &DualChannelVelocityEstimator::watchdogCallback, this);
  
  // Log configuration
  ROS_INFO_STREAM("[DualVelocityEstimator] Left channel: " 
                  << left_channel_.input_topic << " -> velocity: " 
                  << left_channel_.velocity_topic << ", rpy: " 
                  << left_channel_.delta_rpy_topic);
  ROS_INFO_STREAM("[DualVelocityEstimator] Right channel: " 
                  << right_channel_.input_topic << " -> velocity: " 
                  << right_channel_.velocity_topic << ", rpy: " 
                  << right_channel_.delta_rpy_topic);
  ROS_INFO_STREAM("[DualVelocityEstimator] Active velocity: /v_mag_active");
}

void DualChannelVelocityEstimator::loadChannelParameters(
    ros::NodeHandle& channel_nh, Channel& channel, const std::string& name) {
  
  channel.name = name;
  
  // Topic names
  channel_nh.param<std::string>("topic", channel.input_topic, 
                                "/imu_data_" + name);
  channel_nh.param<std::string>("out_topic", channel.velocity_topic, 
                                "/v_mag_" + name);
  channel_nh.param<std::string>("out_topic_RPY", channel.delta_rpy_topic, 
                                "/deltaRPY_" + name);
  
  // Deadbands
  channel_nh.param("deadband_x", channel.deadband_x, defaults::DEADBAND_X);
  channel_nh.param("deadband_y", channel.deadband_y, defaults::DEADBAND_Y);
  channel_nh.param("deadband_z", channel.deadband_z, defaults::DEADBAND_Z);
  
  // ZUPT parameters
  channel_nh.param("zupt_steps", channel.zupt_confirmation_steps, defaults::ZUPT_STEPS);
  channel_nh.param("rpy_zupt_thresh_deg", channel.rpy_stillness_threshold_deg, 
                   defaults::RPY_ZUPT_THRESH_DEG);
  
  // 1D filter parameters
  channel_nh.param("Q", channel.q_1d, defaults::Q);
  channel_nh.param("R", channel.r_1d, defaults::R);
  channel_nh.param("lpf_alpha", channel.lpf_alpha, defaults::LPF_ALPHA);
  
  // 2D filter parameters
  channel_nh.param("q_v", channel.q_velocity, defaults::Q_V);
  channel_nh.param("q_b", channel.q_bias, defaults::Q_B);
  channel_nh.param("r_zupt", channel.r_zupt, defaults::R_ZUPT);
  
  // Time constraints
  channel_nh.param("dt_min", channel.dt_min, defaults::DT_MIN);
  channel_nh.param("dt_max", channel.dt_max, defaults::DT_MAX);
  
  // Create publishers
  channel.velocity_publisher = node_handle_.advertise<std_msgs::Float64>(
      channel.velocity_topic, 10);
  channel.delta_rpy_publisher = node_handle_.advertise<std_msgs::Float64MultiArray>(
      channel.delta_rpy_topic, 10);
  
  channel.last_message_time = ros::Time(0);
}

void DualChannelVelocityEstimator::imuCallback(
    const std_msgs::Float64MultiArray::ConstPtr& msg, Channel* channel) {
  
  channel->last_message_time = ros::Time::now();
  
  // Validate message format
  const auto& data = msg->data;
  if (data.size() != 7) {
    ROS_WARN_THROTTLE(1.0, "[%s] Expected 7 values, got %zu", 
                      channel->name.c_str(), data.size());
    return;
  }
  
  // Parse IMU data: [timestamp_ms, ax, ay, az, roll_deg, pitch_deg, yaw_deg]
  const double timestamp_ms = data[0];
  const double accel_x = data[1];
  const double accel_y = data[2];
  const double accel_z = data[3];
  const float current_rpy[3] = {
      static_cast<float>(data[4]),
      static_cast<float>(data[5]),
      static_cast<float>(data[6])
  };
  
  // Initialize on first message
  if (!channel->has_previous_timestamp) {
    channel->previous_time_ms = timestamp_ms;
    channel->has_previous_timestamp = true;
    
    channel->previous_rpy[0] = current_rpy[0];
    channel->previous_rpy[1] = current_rpy[1];
    channel->previous_rpy[2] = current_rpy[2];
    channel->has_previous_rpy = true;
    return;
  }
  
  // Calculate time step
  const double dt = (timestamp_ms - channel->previous_time_ms) * 1e-3;
  
  if (dt <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "[%s] Non-positive timestep, resyncing", 
                      channel->name.c_str());
    channel->previous_time_ms = timestamp_ms;
    return;
  }
  
  channel->previous_time_ms = timestamp_ms;
  
  // Validate timestep
  if (dt < channel->dt_min || dt > channel->dt_max) {
    ROS_WARN_THROTTLE(1.0, "[%s] dt=%.6f outside valid range [%.6f, %.6f]",
                      channel->name.c_str(), dt, channel->dt_min, channel->dt_max);
    return;
  }
  
  // State transition matrix for velocity-bias model
  const double F00 = 1.0, F01 = -dt;
  const double F10 = 0.0, F11 = 1.0;
  
  // Calculate angular motion
  DeltaRPY angular_change = calculateDeltaRPY(current_rpy, channel->previous_rpy);
  
  // Update stillness counter
  if (angular_change.sum_absolute <= static_cast<float>(channel->rpy_stillness_threshold_deg)) {
    channel->stillness_counter++;
  } else {
    channel->stillness_counter = 0;
  }
  
  const bool is_stationary = (channel->stillness_counter >= channel->zupt_confirmation_steps);
  
  // Filter each axis independently
  filterAxis(channel->x_axis, accel_x, channel->deadband_x, 
             F00, F01, F10, F11, dt, is_stationary, *channel);
  filterAxis(channel->y_axis, accel_y, channel->deadband_y, 
             F00, F01, F10, F11, dt, is_stationary, *channel);
  filterAxis(channel->z_axis, accel_z, channel->deadband_z, 
             F00, F01, F10, F11, dt, is_stationary, *channel);
  
  // Calculate 3D velocity magnitude
  const double vx = channel->x_axis.velocity;
  const double vy = channel->y_axis.velocity;
  const double vz = channel->z_axis.velocity;
  const double velocity_magnitude = std::sqrt(vx*vx + vy*vy + vz*vz) * 100.0;  // m/s to cm/s
  
  // Publish velocity magnitude
  channel->velocity_msg.data = velocity_magnitude;
  channel->velocity_publisher.publish(channel->velocity_msg);
  
  // Publish deltaRPY as [dRoll, dPitch, dYaw, sumAbsolute]
  channel->delta_rpy_msg.data.resize(4);
  channel->delta_rpy_msg.data[0] = angular_change.delta_roll;
  channel->delta_rpy_msg.data[1] = angular_change.delta_pitch;
  channel->delta_rpy_msg.data[2] = angular_change.delta_yaw;
  channel->delta_rpy_msg.data[3] = angular_change.sum_absolute;
  channel->delta_rpy_publisher.publish(channel->delta_rpy_msg);
  
  // Cache results for fusion
  channel->velocity_magnitude = velocity_magnitude;
  channel->delta_rpy_sum = angular_change.sum_absolute;
  channel->output_timestamp = ros::Time::now();
  channel->has_valid_output = true;
  
  // Update fused active velocity
  publishActiveVelocity(dt);
}

void DualChannelVelocityEstimator::filterAxis(
    AxisState& state, double accel_measured, double deadband,
    double F00, double F01, double F10, double F11,
    double dt, bool apply_zupt, const Channel& channel) {
  
  // Apply deadband to measurement
  double accel_deadbanded = accel_measured;
  if (std::abs(accel_deadbanded) < deadband) {
    accel_deadbanded = 0.0;
  }
  
  // 1D Kalman filter for acceleration smoothing
  state.accel_variance += channel.q_1d;
  const double kalman_gain = state.accel_variance / (state.accel_variance + channel.r_1d);
  state.accel_filtered += kalman_gain * (accel_deadbanded - state.accel_filtered);
  state.accel_variance *= (1.0 - kalman_gain);
  
  // Light low-pass filter for integration
  double accel_for_integration = accel_measured;
  if (state.has_prev_accel) {
    accel_for_integration = channel.lpf_alpha * state.prev_accel_lpf + 
                           (1.0 - channel.lpf_alpha) * accel_measured;
  }
  state.prev_accel_lpf = accel_for_integration;
  state.has_prev_accel = true;
  
  // Integrate velocity (with bias compensation)
  state.velocity += (accel_for_integration - state.bias) * dt;
  
  // Predict covariance (2x2 matrix)
  const double temp00 = F00 * state.P00 + F01 * state.P10;
  const double temp01 = F00 * state.P01 + F01 * state.P11;
  const double temp10 = F10 * state.P00 + F11 * state.P10;
  const double temp11 = F10 * state.P01 + F11 * state.P11;
  
  state.P00 = temp00 * F00 + temp01 * F01 + channel.q_velocity * dt * dt;
  state.P01 = temp00 * F10 + temp01 * F11;
  state.P10 = temp10 * F00 + temp11 * F01;
  state.P11 = temp10 * F10 + temp11 * F11 + channel.q_bias * dt;
  
  // Apply ZUPT (Zero velocity UPdaTe) when stationary
  if (apply_zupt) {
    const double innovation = -state.velocity;  // Expected velocity is zero
    const double innovation_variance = state.P00 + channel.r_zupt;
    
    const double gain_velocity = state.P00 / innovation_variance;
    const double gain_bias = state.P10 / innovation_variance;
    
    state.velocity += gain_velocity * innovation;
    state.bias += gain_bias * innovation;
    
    // Update covariance
    const double P00_old = state.P00;
    const double P01_old = state.P01;
    const double P10_old = state.P10;
    const double P11_old = state.P11;
    
    state.P00 = (1.0 - gain_velocity) * P00_old;
    state.P01 = (1.0 - gain_velocity) * P01_old;
    state.P10 = P10_old - gain_bias * P00_old;
    state.P11 = P11_old - gain_bias * P01_old;
  }
}

void DualChannelVelocityEstimator::publishActiveVelocity(double fallback_dt) {
  double dt = fallback_dt;
  
  // Calculate time since last active velocity publish
  const ros::Time now = ros::Time::now();
  if (!has_active_publish_time_) {
    last_active_publish_time_ = now;
    has_active_publish_time_ = true;
  } else {
    dt = (now - last_active_publish_time_).toSec();
    last_active_publish_time_ = now;
    if (dt <= 1e-6) dt = fallback_dt;
  }
  
  // Check data freshness
  auto isDataFresh = [&](const Channel& ch) {
    if (!ch.has_valid_output) return false;
    return (now - ch.output_timestamp).toSec() <= data_freshness_threshold_sec_;
  };
  
  const bool left_fresh = isDataFresh(left_channel_);
  const bool right_fresh = isDataFresh(right_channel_);
  
  // If neither channel has fresh data, decay to zero
  if (!left_fresh && !right_fresh) {
    const double target = 0.0;
    const double alpha = (target > active_velocity_output_) ? alpha_rising_ : alpha_falling_;
    active_velocity_output_ += alpha * (target - active_velocity_output_);
    
    active_velocity_msg_.data = active_velocity_output_;
    active_velocity_publisher_.publish(active_velocity_msg_);
    return;
  }
  
  // Get velocities and motion indicators
  const double vel_left = left_fresh ? left_channel_.velocity_magnitude : 0.0;
  const double vel_right = right_fresh ? right_channel_.velocity_magnitude : 0.0;
  
  const double motion_left = left_fresh ? left_channel_.delta_rpy_sum : 0.0;
  const double motion_right = right_fresh ? right_channel_.delta_rpy_sum : 0.0;
  
  const bool left_moving = (motion_left >= motion_threshold_deg_);
  const bool right_moving = (motion_right >= motion_threshold_deg_);
  
  // Fuse velocities based on motion detection
  double target_velocity = 0.0;
  
  if (left_moving || right_moving) {
    if (fusion_mode_ == 0) {
      // MAX fusion: use whichever foot is moving faster
      target_velocity = std::max(vel_left, vel_right);
    } else {
      // Weighted blend based on amount of angular motion
      const double weight_left_raw = std::max(0.0, motion_left - motion_threshold_deg_);
      const double weight_right_raw = std::max(0.0, motion_right - motion_threshold_deg_);
      const double total_weight = weight_left_raw + weight_right_raw + 1e-9;
      const double weight_left = weight_left_raw / total_weight;
      
      target_velocity = weight_left * vel_left + (1.0 - weight_left) * vel_right;
    }
  } else {
    target_velocity = 0.0;
  }
  
  // Envelope follower: asymmetric rise/fall rates
  const double alpha = (target_velocity > active_velocity_output_) ? alpha_rising_ : alpha_falling_;
  double next_velocity = active_velocity_output_ + alpha * (target_velocity - active_velocity_output_);
  
  // Limit maximum velocity drop rate (prevents unrealistic jumps)
  if (max_velocity_drop_rate_ > 0.0 && dt > 1e-6) {
    const double max_drop = max_velocity_drop_rate_ * dt;
    if (next_velocity < active_velocity_output_ - max_drop) {
      next_velocity = active_velocity_output_ - max_drop;
    }
  }
  
  active_velocity_output_ = next_velocity;
  active_velocity_msg_.data = active_velocity_output_;
  active_velocity_publisher_.publish(active_velocity_msg_);
}

void DualChannelVelocityEstimator::watchdogCallback(const ros::TimerEvent&) {
  checkChannelHealth(left_channel_);
  checkChannelHealth(right_channel_);
}

void DualChannelVelocityEstimator::checkChannelHealth(const Channel& channel) {
  if (channel.last_message_time.isZero()) {
    ROS_WARN_THROTTLE(2.0, "[%s] No IMU messages received yet", 
                      channel.name.c_str());
    return;
  }
  
  const double age = (ros::Time::now() - channel.last_message_time).toSec();
  if (age > 2.0) {
    ROS_WARN_THROTTLE(2.0, "[%s] No IMU messages for %.1f seconds", 
                      channel.name.c_str(), age);
  }
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_calculator_dual");
  ros::NodeHandle private_nh("~");
  
  DualChannelVelocityEstimator node(private_nh);
  
  ROS_INFO("[DualVelocityEstimator] Node started, spinning...");
  ros::spin();
  
  return 0;
}