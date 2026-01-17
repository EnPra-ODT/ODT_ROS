#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <limits>
#include <string>

float g_prev_RPY[3] = {0.0f};

struct AxisState {
    int still_count = 0;

    double a_hat = 0.0;
    double P_1d  = 1.0;

    bool   has_light_prev = false;
    double a_light_prev   = 0.0;

    double v2 = 0.0;
    double b2 = 0.0;

    double P00 = 1.0, P01 = 0.0, P10 = 0.0, P11 = 1.0;
};

class ImuKfLogger {
public:
    ImuKfLogger(ros::NodeHandle &nh) : nh_(nh){
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


        nh_.param<std::string>("out_topic", out_topic_, std::string("v_mag"));
        vmag_pub_ = nh_.advertise<std_msgs::Float64>(out_topic_, 10);

        nh_.param<std::string>("out_topic_RPY", out_topic_RPY_, std::string("deltaRPY"));
        deltaRPY_pub_ = nh.advertise<std_msgs::Float64>(out_topic_RPY_,10);

        sub_ = nh_.subscribe(topic_, 50, &ImuKfLogger::cb, this);

        last_msg_time_ = ros::Time(0);

        watchdog_ = nh_.createTimer(
            ros::Duration(1.0),
            &ImuKfLogger::watchdogCb,
            this
        );

        ROS_INFO_STREAM("Subscribed to topic: " << topic_);
        ROS_INFO_STREAM("Publishing Float64 on: " << out_topic_);
    }

private:
    void watchdogCb(const ros::TimerEvent &){
        if (last_msg_time_.isZero()) {
            ROS_WARN_THROTTLE(2.0, "No IMU messages received yet.");
            return;
        }
        if ((ros::Time::now() - last_msg_time_).toSec() > 2.0) {
            ROS_WARN_THROTTLE(2.0, "No IMU messages received in >2s.");
        }
    }
    static inline float wrapDiffDeg(float cur, float prev){
        float d = cur - prev;
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return d;
    }

    float calcDeltaRPY(const float* cur, float* prev){
        const float deltaRoll  = wrapDiffDeg(cur[0], prev[0]);
        const float deltaPitch = wrapDiffDeg(cur[1], prev[1]);
        const float deltaYaw   = wrapDiffDeg(cur[2], prev[2]);

        prev[0] = cur[0];
        prev[1] = cur[1];
        prev[2] = cur[2];

        return std::fabs(deltaRoll) + std::fabs(deltaPitch) + std::fabs(deltaYaw);
    }

    void cb(const std_msgs::Float64MultiArray::ConstPtr &msg){
        last_msg_time_ = ros::Time::now();
        const auto &d = msg->data;
        if (d.size() != 7) return;

        double t_ms = d[0];
        double ax   = d[1];
        double ay   = d[2];
        double az   = d[3];
        float current_angle[3] = {static_cast<float>(d[4]), static_cast<float>(d[5]), static_cast<float>(d[6])};

        // --- deltaRPY (deg) ---
        float deltaRPY = 0.0f;
        if (!have_prev_rpy_) {
            // initialize previous angles on first message
            g_prev_RPY[0] = current_angle[0];
            g_prev_RPY[1] = current_angle[1];
            g_prev_RPY[2] = current_angle[2];
            have_prev_rpy_ = true;
            deltaRPY = 0.0f;
        } else {
            deltaRPY = calcDeltaRPY(current_angle, g_prev_RPY);
        }

        // --- Stillness detection using deltaRPY ---
        if (deltaRPY <= static_cast<float>(rpy_zupt_thresh_deg_)) rpy_still_count_++;
        else rpy_still_count_ = 0;

        const bool still = (rpy_still_count_ >= zupt_steps_);

        processAxis(x_, ax, deadband_x_, F00, F01, F10, F11, dt, still);
        processAxis(y_, ay, deadband_y_, F00, F01, F10, F11, dt, still);
        processAxis(z_, az, deadband_z_, F00, F01, F10, F11, dt, still);

        const double vx = x_.v2;
        const double vy = y_.v2;
        const double vz = z_.v2;
        const double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz) * 100.0; // cm/s

        vmag_msg_.data = v_mag;
        vmag_pub_.publish(vmag_msg_);
        deltaRPY_msg_.data = deltaRPY;
        deltaRPY_pub_.publish(deltaRPY_msg_);

    }

    double processAxis(AxisState &S, double a_in, double deadband, double F00, double F01, double F10, double F11, double dt, bool still){
        double z = a_in;
        if (std::abs(z) < deadband) z = 0.0;

        S.P_1d += Q_;
        double K = S.P_1d / (S.P_1d + R_);
        S.a_hat += K * (z - S.a_hat);
        S.P_1d  *= (1.0 - K);

        double a_light;
        if (!S.has_light_prev) {
            a_light = a_in;
            S.has_light_prev = true;
        } else {
            a_light = lpf_alpha_ * S.a_light_prev + (1.0 - lpf_alpha_) * a_in;
        }
        S.a_light_prev = a_light;

        const double u = a_light;

        const bool still = (S.still_count >= zupt_steps_);
        S.v2 += (u - S.b2) * dt;

        double A00 = F00 * S.P00 + F01 * S.P10;
        double A01 = F00 * S.P01 + F01 * S.P11;
        double A10 = F10 * S.P00 + F11 * S.P10;
        double A11 = F10 * S.P01 + F11 * S.P11;

        double P00p = A00 * F00 + A01 * F01 + q_v_ * dt * dt;
        double P01p = A00 * F10 + A01 * F11;
        double P10p = A10 * F00 + A11 * F01;
        double P11p = A10 * F10 + A11 * F11 + q_b_ * dt;

        S.P00 = P00p; S.P01 = P01p; S.P10 = P10p; S.P11 = P11p;

        // ZUPT update
        if (still) {
            double y = -S.v2;
            double Szz = S.P00 + r_zupt_;

            double K0 = S.P00 / Szz;
            double K1 = S.P10 / Szz;

            S.v2 += K0 * y;
            S.b2 += K1 * y;

            const double P00_old = S.P00;
            const double P01_old = S.P01;
            const double P10_old = S.P10;
            const double P11_old = S.P11;

            const double P00_new = (1.0 - K0) * P00_old;
            const double P01_new = (1.0 - K0) * P01_old;
            const double P10_new = P10_old - K1 * P00_old;
            const double P11_new = P11_old - K1 * P01_old;

            S.P00 = P00_new;
            S.P01 = P01_new;
            S.P10 = P10_new;
            S.P11 = P11_new;
        }

        return S.a_hat;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  vmag_pub_;
    ros::Publisher deltaRPY_pub_;
    std_msgs::Float64 vmag_msg_;
    std_msgs::Float64 deltaRPY_msg_;

    ros::Timer watchdog_;
    ros::Time last_msg_time_;

    std::string topic_;
    std::string out_topic_;
    std::string out_topic_RPY_;

    double deadband_x_, deadband_y_, deadband_z_;
    int zupt_steps_;

    double Q_, R_;
    double lpf_alpha_;
    double q_v_, q_b_, r_zupt_;
    double dt_min_, dt_max_;

    bool have_prev_time_ = false;
    double t_prev_ms_ = 0.0;

    double rpy_zupt_thresh_deg_ = 3.0;  // still if deltaRPY <= this
    int    rpy_still_count_ = 0;
    bool   have_prev_rpy_ = false;


    AxisState x_, y_, z_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_kf_logger");
    ros::NodeHandle nh("~");

    ImuKfLogger node(nh);
    ros::spin();
    return 0;
}
