#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <boost/bind.hpp>
#include <cmath>
#include <string>

// ----------------- Per-axis state -----------------
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

// ----------------- Helpers -----------------
static inline float wrapDiffDeg(float cur, float prev){
    float d = cur - prev;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static inline float calcDeltaRPY(const float cur[3], float prev[3]){
    const float dR = wrapDiffDeg(cur[0], prev[0]);
    const float dP = wrapDiffDeg(cur[1], prev[1]);
    const float dY = wrapDiffDeg(cur[2], prev[2]);

    prev[0] = cur[0];
    prev[1] = cur[1];
    prev[2] = cur[2];

    return std::fabs(dR) + std::fabs(dP) + std::fabs(dY);
}

// ----------------- Node -----------------
class VelocityCalculatorDual {
public:
    VelocityCalculatorDual(ros::NodeHandle& pnh) : pnh_(pnh) {
        ros::NodeHandle lnh(pnh_, "left");
        ros::NodeHandle rnh(pnh_, "right");

        loadChannelParams(lnh, left_,  "left");
        loadChannelParams(rnh, right_, "right");

        // Active-selection params (node-level)
        pnh_.param("active_stale_sec",  active_stale_sec_,  0.20);
        pnh_.param("move_thresh_deg",   move_thresh_deg_,   1.0);
        pnh_.param("switch_margin_deg", switch_margin_deg_, 0.5);
        pnh_.param("switch_hold_steps", switch_hold_steps_, 3);

        active_vmag_pub_ = pnh_.advertise<std_msgs::Float64>("/v_mag_active", 10);

        left_.sub = pnh_.subscribe<std_msgs::Float64MultiArray>(
            left_.topic, 50,
            boost::bind(&VelocityCalculatorDual::cb, this, _1, &left_)
        );
        right_.sub = pnh_.subscribe<std_msgs::Float64MultiArray>(
            right_.topic, 50,
            boost::bind(&VelocityCalculatorDual::cb, this, _1, &right_)
        );

        watchdog_ = pnh_.createTimer(
            ros::Duration(1.0),
            &VelocityCalculatorDual::watchdogCb,
            this
        );

        ROS_INFO_STREAM("[velocity_calculator_dual] Left  topic: " << left_.topic
                        << " -> v: " << left_.out_topic_vmag
                        << " rpy: " << left_.out_topic_rpy);
        ROS_INFO_STREAM("[velocity_calculator_dual] Right topic: " << right_.topic
                        << " -> v: " << right_.out_topic_vmag
                        << " rpy: " << right_.out_topic_rpy);
        ROS_INFO_STREAM("[velocity_calculator_dual] Active velocity topic: /v_mag_active");
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
        std_msgs::Float64 rpy_msg;

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
        bool     has_latest = false;
        ros::Time latest_stamp;
        double   vmag = 0.0;       // cm/s
        double   delta_rpy = 0.0;  // deg-sum
    };

    enum class ActiveSide { NONE=0, LEFT=1, RIGHT=2 };

    void loadChannelParams(ros::NodeHandle& nhc, Channel& C, const std::string& label){
        C.label = label;

        nhc.param<std::string>("topic", C.topic, std::string("/imu_data_" + label));
        nhc.param<std::string>("out_topic", C.out_topic_vmag, std::string("/v_mag_" + label));
        nhc.param<std::string>("out_topic_RPY", C.out_topic_rpy, std::string("/deltaRPY_" + label));

        nhc.param("deadband_x", C.deadband_x, 0.15);
        nhc.param("deadband_y", C.deadband_y, 0.15);
        nhc.param("deadband_z", C.deadband_z, 0.30);
        nhc.param("zupt_steps", C.zupt_steps, 3);

        nhc.param("Q", C.Q, 0.05);
        nhc.param("R", C.R, 0.20);
        nhc.param("lpf_alpha", C.lpf_alpha, 0.3);

        nhc.param("q_v", C.q_v, 0.5);
        nhc.param("q_b", C.q_b, 0.01);
        nhc.param("r_zupt", C.r_zupt, 1e-4);

        nhc.param("dt_min", C.dt_min, 0.001);
        nhc.param("dt_max", C.dt_max, 0.10);

        nhc.param("rpy_zupt_thresh_deg", C.rpy_zupt_thresh_deg, 5.0);

        C.vmag_pub = pnh_.advertise<std_msgs::Float64>(C.out_topic_vmag, 10);
        C.rpy_pub  = pnh_.advertise<std_msgs::Float64>(C.out_topic_rpy, 10);

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
    void publishActive(){
        const ros::Time now = ros::Time::now();

        auto isFresh = [&](const Channel& C){
            if (!C.has_latest) return false;
            return (now - C.latest_stamp).toSec() <= active_stale_sec_;
        };

        const bool Lfresh = isFresh(left_);
        const bool Rfresh = isFresh(right_);

        if (Lfresh && !Rfresh) {
            active_side_ = ActiveSide::LEFT;
            active_vmag_msg_.data = left_.vmag;
            active_vmag_pub_.publish(active_vmag_msg_);
            return;
        }
        if (!Lfresh && Rfresh) {
            active_side_ = ActiveSide::RIGHT;
            active_vmag_msg_.data = right_.vmag;
            active_vmag_pub_.publish(active_vmag_msg_);
            return;
        }
        if (!Lfresh && !Rfresh) {
            active_side_ = ActiveSide::NONE;
            active_vmag_msg_.data = 0.0;
            active_vmag_pub_.publish(active_vmag_msg_);
            return;
        }

        const bool Lmoving = (left_.delta_rpy  >= move_thresh_deg_);
        const bool Rmoving = (right_.delta_rpy >= move_thresh_deg_);

        if (Lmoving && !Rmoving) {
            requestSwitch(ActiveSide::LEFT);
            publishCurrentActive();
            return;
        }
        if (!Lmoving && Rmoving) {
            requestSwitch(ActiveSide::RIGHT);
            publishCurrentActive();
            return;
        }

        if (!Lmoving && !Rmoving) {
            if (active_side_ == ActiveSide::NONE) active_side_ = ActiveSide::LEFT;
            publishCurrentActive();
            return;
        }

        // Both moving: deltaRPY wins; tie-break with vmag
        const double dL = left_.delta_rpy;
        const double dR = right_.delta_rpy;

        ActiveSide winner;
        if (std::fabs(dL - dR) >= switch_margin_deg_) {
            winner = (dL > dR) ? ActiveSide::LEFT : ActiveSide::RIGHT;
        } else {
            winner = (left_.vmag >= right_.vmag) ? ActiveSide::LEFT : ActiveSide::RIGHT;
        }

        requestSwitch(winner);
        publishCurrentActive();
    }

    void publishCurrentActive(){
        if (active_side_ == ActiveSide::RIGHT) active_vmag_msg_.data = right_.vmag;
        else {
            if (active_side_ == ActiveSide::NONE) active_side_ = ActiveSide::LEFT;
            active_vmag_msg_.data = left_.vmag;
        }
        active_vmag_pub_.publish(active_vmag_msg_);
    }

    void requestSwitch(ActiveSide desired){
        if (active_side_ == ActiveSide::NONE) {
            active_side_ = desired;
            switch_count_ = 0;
            return;
        }
        if (desired == active_side_) {
            switch_count_ = 0;
            return;
        }
        switch_count_++;
        if (switch_count_ >= switch_hold_steps_) {
            active_side_ = desired;
            switch_count_ = 0;
        }
    }

    // ---------- Callback ----------
    void cb(const std_msgs::Float64MultiArray::ConstPtr& msg, Channel* C){
        C->last_msg_time = ros::Time::now();

        const auto& d = msg->data;
        if (d.size() != 7) return;

        const double t_ms = d[0];
        const double ax   = d[1];
        const double ay   = d[2];
        const double az   = d[3];

        const float current_angle[3] = {
            static_cast<float>(d[4]),
            static_cast<float>(d[5]),
            static_cast<float>(d[6])
        };

        if (!C->have_prev_time) {
            C->t_prev_ms = t_ms;
            C->have_prev_time = true;

            C->prev_rpy[0] = current_angle[0];
            C->prev_rpy[1] = current_angle[1];
            C->prev_rpy[2] = current_angle[2];
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

        float deltaRPY = calcDeltaRPY(current_angle, C->prev_rpy);

        if (deltaRPY <= static_cast<float>(C->rpy_zupt_thresh_deg)) C->rpy_still_count++;
        else C->rpy_still_count = 0;

        const bool still_global = (C->rpy_still_count >= C->zupt_steps);

        processAxis(C->x, ax, C->deadband_x, F00, F01, F10, F11, dt, still_global, *C);
        processAxis(C->y, ay, C->deadband_y, F00, F01, F10, F11, dt, still_global, *C);
        processAxis(C->z, az, C->deadband_z, F00, F01, F10, F11, dt, still_global, *C);

        const double vx = C->x.v2;
        const double vy = C->y.v2;
        const double vz = C->z.v2;

        const double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz) * 100.0; // cm/s

        C->vmag_msg.data = v_mag;
        C->vmag_pub.publish(C->vmag_msg);

        C->rpy_msg.data = deltaRPY;
        C->rpy_pub.publish(C->rpy_msg);

        // cache latest
        C->vmag = v_mag;
        C->delta_rpy = deltaRPY;
        C->latest_stamp = ros::Time::now();
        C->has_latest = true;

        // choose which IMU is "active moving" and publish it
        publishActive();
    }

    double processAxis(
        AxisState& S,
        double a_in,
        double deadband,
        double F00, double F01, double F10, double F11,
        double dt,
        bool still_global,
        const Channel& C
    ){
        double z = a_in;
        if (std::abs(z) < deadband) z = 0.0;

        S.P_1d += C.Q;
        const double K = S.P_1d / (S.P_1d + C.R);
        S.a_hat += K * (z - S.a_hat);
        S.P_1d  *= (1.0 - K);

        double a_light;
        if (!S.has_light_prev) {
            a_light = a_in;
            S.has_light_prev = true;
        } else {
            a_light = C.lpf_alpha * S.a_light_prev + (1.0 - C.lpf_alpha) * a_in;
        }
        S.a_light_prev = a_light;

        const double u = a_light;

        S.v2 += (u - S.b2) * dt;

        const double A00 = F00 * S.P00 + F01 * S.P10;
        const double A01 = F00 * S.P01 + F01 * S.P11;
        const double A10 = F10 * S.P00 + F11 * S.P10;
        const double A11 = F10 * S.P01 + F11 * S.P11;

        const double P00p = A00 * F00 + A01 * F01 + C.q_v * dt * dt;
        const double P01p = A00 * F10 + A01 * F11;
        const double P10p = A10 * F00 + A11 * F01;
        const double P11p = A10 * F10 + A11 * F11 + C.q_b * dt;

        S.P00 = P00p; S.P01 = P01p; S.P10 = P10p; S.P11 = P11p;

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

        return S.a_hat;
    }

private:
    ros::NodeHandle pnh_;
    ros::Timer watchdog_;

    Channel left_;
    Channel right_;

    // Active-selection state (node-level)
    ros::Publisher active_vmag_pub_;
    std_msgs::Float64 active_vmag_msg_;

    ActiveSide active_side_ = ActiveSide::NONE;

    double active_stale_sec_   = 0.20;
    double move_thresh_deg_    = 1.0;
    double switch_margin_deg_  = 0.5;
    int    switch_hold_steps_  = 3;
    int    switch_count_       = 0;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "velocity_calculator_dual");
    ros::NodeHandle pnh("~");

    VelocityCalculatorDual node(pnh);
    ros::spin();
    return 0;
}
