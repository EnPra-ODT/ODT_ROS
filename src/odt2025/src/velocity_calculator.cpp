/*velocity_calculator.cpp
 * IMU Kalman Filter Logger Node
 * 
 * PURPOSE:
 *   Processes IMU acceleration data using Kalman filtering to estimate velocity.
 *   Implements Zero-velocity Update (ZUPT) detection using orientation (RPY) changes
 *   to correct for drift in velocity estimates.
 * 
 * SUBSCRIBES TO:
 *   - /imu_data_left (std_msgs/Float64MultiArray) [default, configurable via "topic" param]
 *     Format: [timestamp_ms, ax, ay, az, roll, pitch, yaw]
 *     - timestamp_ms: timestamp in milliseconds
 *     - ax, ay, az: linear acceleration in m/s² (x, y, z axes)
 *     - roll, pitch, yaw: orientation angles in degrees
 * 
 * PUBLISHES:
 *   - v_mag (std_msgs/Float64) [default, configurable via "out_topic" param]
 *     Velocity magnitude in cm/s computed from 3D velocity estimate
 * 
 *   - deltaRPY (std_msgs/Float64) [default, configurable via "out_topic_RPY" param]
 *     Sum of absolute angular changes (|ΔRoll| + |ΔPitch| + |ΔYaw|) in degrees
 * 
 * FEATURES:
 *   - Per-axis Kalman filtering with deadband noise rejection
 *   - Low-pass filtering on acceleration input
 *   - 2-state Kalman filter per axis (velocity + bias estimation)
 *   - ZUPT (Zero-velocity Update) when orientation is stable
 *   - Watchdog timer to detect missing IMU messages
 * 
 * KEY PARAMETERS (ROS params):
 *   - deadband_x/y/z: Acceleration deadband thresholds (m/s²)
 *   - zupt_steps: Number of consecutive still frames needed for ZUPT
 *   - rpy_zupt_thresh_deg: Max deltaRPY for "stillness" detection (degrees)
 *   - Q, R: 1D Kalman filter process/measurement noise
 *   - q_v, q_b: Process noise for velocity and bias in 2-state filter
 *   - r_zupt: Measurement noise for ZUPT correction
 *   - lpf_alpha: Low-pass filter coefficient (0-1)
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

// Global storage for previous RPY angles (used for delta computation)
float g_prev_RPY[3] = {0.0f};

// State variables for per-axis filtering
struct AxisState {
    int still_count = 0;

    // 1D Kalman filter state
    double a_hat = 0.0;  // Filtered acceleration estimate
    double P_1d  = 1.0;  // Error covariance for 1D filter

    // Low-pass filter state
    bool   has_light_prev = false;
    double a_light_prev   = 0.0;

    // 2-state Kalman filter: [velocity, bias]
    double v2 = 0.0;  // Velocity estimate
    double b2 = 0.0;  // Bias estimate

    // 2x2 error covariance matrix
    double P00 = 1.0, P01 = 0.0;
    double P10 = 0.0, P11 = 1.0;
};

class ImuKfLogger {
public:
    ImuKfLogger(ros::NodeHandle &nh) : nh_(nh) {
        // Load parameters
        nh_.param<std::string>("topic", topic_, std::string("/imu_data_left"));

        nh_.param("deadband_x", deadband_x_, 0.15);
        nh_.param("deadband_y", deadband_y_, 0.15);
        nh_.param("deadband_z", deadband_z_, 0.30);
        nh_.param("zupt_steps", zupt_steps_, 5);

        nh_.param("Q", Q_, 0.05);
        nh_.param("R", R_, 0.20);

        nh_.param("lpf_alpha", lpf_alpha_, 0.3);

        nh_.param("q_v", q_v_, 0.5);
        nh_.param("q_b", q_b_, 0.01);
        nh_.param("r_zupt", r_zupt_, 1e-4);

        nh_.param("dt_min", dt_min_, 0.001);
        nh_.param("dt_max", dt_max_, 0.20);

        nh_.param("rpy_zupt_thresh_deg", rpy_zupt_thresh_deg_, 3.0);

        // Set up publishers
        nh_.param<std::string>("out_topic", out_topic_, std::string("v_mag"));
        vmag_pub_ = nh_.advertise<std_msgs::Float64>(out_topic_, 10);

        nh_.param<std::string>("out_topic_RPY", out_topic_RPY_, std::string("deltaRPY"));
        deltaRPY_pub_ = nh.advertise<std_msgs::Float64>(out_topic_RPY_, 10);

        // Set up subscriber
        sub_ = nh_.subscribe(topic_, 50, &ImuKfLogger::cb, this);

        last_msg_time_ = ros::Time(0);

        // Watchdog timer to detect missing messages
        watchdog_ = nh_.createTimer(
            ros::Duration(1.0),
            &ImuKfLogger::watchdogCb,
            this
        );

        ROS_INFO_STREAM("Subscribed to topic: " << topic_);
        ROS_INFO_STREAM("Publishing velocity on: " << out_topic_);
        ROS_INFO_STREAM("Publishing deltaRPY on: " << out_topic_RPY_);
    }

private:
    // Watchdog callback to warn if IMU messages stop arriving
    void watchdogCb(const ros::TimerEvent &) {
        if (last_msg_time_.isZero()) {
            ROS_WARN_THROTTLE(2.0, "No IMU messages received yet.");
            return;
        }
        if ((ros::Time::now() - last_msg_time_).toSec() > 2.0) {
            ROS_WARN_THROTTLE(2.0, "No IMU messages received in >2s.");
        }
    }

    // Compute angle difference handling wraparound at ±180°
    static inline float wrapDiffDeg(float cur, float prev) {
        float d = cur - prev;
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return d;
    }

    // Calculate total angular change from previous RPY
    float calcDeltaRPY(const float* cur, float* prev) {
        const float deltaRoll  = wrapDiffDeg(cur[0], prev[0]);
        const float deltaPitch = wrapDiffDeg(cur[1], prev[1]);
        const float deltaYaw   = wrapDiffDeg(cur[2], prev[2]);

        // Update previous values
        prev[0] = cur[0];
        prev[1] = cur[1];
        prev[2] = cur[2];

        return std::fabs(deltaRoll) + std::fabs(deltaPitch) + std::fabs(deltaYaw);
    }

    // Main callback for incoming IMU messages
    void cb(const std_msgs::Float64MultiArray::ConstPtr &msg) {
        last_msg_time_ = ros::Time::now();
        const auto &d = msg->data;
        
        // Validate message format
        if (d.size() != 7) return;

        // Extract data
        const double t_ms = d[0];
        const double ax   = d[1];
        const double ay   = d[2];
        const double az   = d[3];

        const float current_angle[3] = {
            static_cast<float>(d[4]),  // roll
            static_cast<float>(d[5]),  // pitch
            static_cast<float>(d[6])   // yaw
        };

        // ---- Compute time step ----
        if (!have_prev_time_) {
            t_prev_ms_ = t_ms;
            have_prev_time_ = true;

            // Initialize RPY tracking
            g_prev_RPY[0] = current_angle[0];
            g_prev_RPY[1] = current_angle[1];
            g_prev_RPY[2] = current_angle[2];
            have_prev_rpy_ = true;
            return;
        }

        const double dt = (t_ms - t_prev_ms_) * 1e-3;  // Convert ms to seconds
        if (!(dt > 0.0)) {
            ROS_WARN_THROTTLE(1.0, "Non-positive dt (t_ms reset/wrap?). Resyncing time.");
            t_prev_ms_ = t_ms;
            return;
        }
        t_prev_ms_ = t_ms;

        // Reject unrealistic time steps
        if (dt < dt_min_ || dt > dt_max_) {
            ROS_WARN_THROTTLE(1.0, "dt=%.6f rejected (min=%.6f max=%.6f).", dt, dt_min_, dt_max_);
            return;
        }

        // State transition matrix elements for velocity integration
        const double F00 = 1.0, F01 = -dt;
        const double F10 = 0.0, F11 =  1.0;

        // ---- Compute deltaRPY and detect stillness ----
        float deltaRPY = 0.0f;
        if (!have_prev_rpy_) {
            g_prev_RPY[0] = current_angle[0];
            g_prev_RPY[1] = current_angle[1];
            g_prev_RPY[2] = current_angle[2];
            have_prev_rpy_ = true;
            deltaRPY = 0.0f;
        } else {
            deltaRPY = calcDeltaRPY(current_angle, g_prev_RPY);
        }

        // Track consecutive still frames
        if (deltaRPY <= static_cast<float>(rpy_zupt_thresh_deg_)) {
            rpy_still_count_++;
        } else {
            rpy_still_count_ = 0;
        }

        const bool still_global = (rpy_still_count_ >= zupt_steps_);

        // ---- Process each axis with Kalman filtering ----
        processAxis(x_, ax, deadband_x_, F00, F01, F10, F11, dt, still_global);
        processAxis(y_, ay, deadband_y_, F00, F01, F10, F11, dt, still_global);
        processAxis(z_, az, deadband_z_, F00, F01, F10, F11, dt, still_global);

        // ---- Compute and publish velocity magnitude ----
        const double vx = x_.v2;
        const double vy = y_.v2;
        const double vz = z_.v2;
        const double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz) * 100.0;  // Convert to cm/s

        vmag_msg_.data = v_mag;
        vmag_pub_.publish(vmag_msg_);

        deltaRPY_msg_.data = deltaRPY;
        deltaRPY_pub_.publish(deltaRPY_msg_);
    }

    // Process single axis with Kalman filtering and ZUPT
    double processAxis(AxisState &S, double a_in, double deadband, 
                      double F00, double F01, double F10, double F11, 
                      double dt, bool still_global) {
        // Apply deadband to remove noise
        double z = a_in;
        if (std::abs(z) < deadband) z = 0.0;

        // ---- 1D Kalman filter for acceleration ----
        S.P_1d += Q_;  // Predict covariance
        const double K = S.P_1d / (S.P_1d + R_);  // Kalman gain
        S.a_hat += K * (z - S.a_hat);  // Update estimate
        S.P_1d  *= (1.0 - K);  // Update covariance

        // ---- Low-pass filter on raw acceleration ----
        double a_light;
        if (!S.has_light_prev) {
            a_light = a_in;
            S.has_light_prev = true;
        } else {
            a_light = lpf_alpha_ * S.a_light_prev + (1.0 - lpf_alpha_) * a_in;
        }
        S.a_light_prev = a_light;

        // ---- Integrate velocity (with bias correction) ----
        const double u = a_light;
        S.v2 += (u - S.b2) * dt;

        // ---- Predict covariance for 2-state filter ----
        const double A00 = F00 * S.P00 + F01 * S.P10;
        const double A01 = F00 * S.P01 + F01 * S.P11;
        const double A10 = F10 * S.P00 + F11 * S.P10;
        const double A11 = F10 * S.P01 + F11 * S.P11;

        const double P00p = A00 * F00 + A01 * F01 + q_v_ * dt * dt;
        const double P01p = A00 * F10 + A01 * F11;
        const double P10p = A10 * F00 + A11 * F01;
        const double P11p = A10 * F10 + A11 * F11 + q_b_ * dt;

        S.P00 = P00p; S.P01 = P01p; S.P10 = P10p; S.P11 = P11p;

        // ---- Apply ZUPT correction if still ----
        if (still_global) {
            const double y = -S.v2;  // Innovation (measurement - prediction)
            const double Szz = S.P00 + r_zupt_;  // Innovation covariance

            // Kalman gains
            const double K0 = S.P00 / Szz;
            const double K1 = S.P10 / Szz;

            // Update state estimates
            S.v2 += K0 * y;
            S.b2 += K1 * y;

            // Update covariance
            const double P00_old = S.P00;
            const double P01_old = S.P01;
            const double P10_old = S.P10;
            const double P11_old = S.P11;

            S.P00 = (1.0 - K0) * P00_old;
            S.P01 = (1.0 - K0) * P01_old;
            S.P10 = P10_old - K1 * P00_old;
            S.P11 = P11_old - K1 * P01_old;
        }

        return S.a_hat;
    }

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  vmag_pub_;
    ros::Publisher  deltaRPY_pub_;
    std_msgs::Float64 vmag_msg_;
    std_msgs::Float64 deltaRPY_msg_;

    ros::Timer watchdog_;
    ros::Time last_msg_time_;

    // Configuration
    std::string topic_;
    std::string out_topic_;
    std::string out_topic_RPY_;

    double deadband_x_, deadband_y_, deadband_z_;
    int zupt_steps_;

    double Q_, R_;
    double lpf_alpha_;
    double q_v_, q_b_, r_zupt_;
    double dt_min_, dt_max_;

    // Time tracking
    bool have_prev_time_ = false;
    double t_prev_ms_ = 0.0;

    // RPY stillness detection
    double rpy_zupt_thresh_deg_ = 3.0;
    int    rpy_still_count_ = 0;
    bool   have_prev_rpy_ = false;

    // Per-axis filter states
    AxisState x_, y_, z_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_kf_logger");
    ros::NodeHandle nh("~");

    ImuKfLogger node(nh);
    ros::spin();
    return 0;
}