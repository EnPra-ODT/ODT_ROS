#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

static inline void quatNormalize(double &w, double &x, double &y, double &z)
{
    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n < 1e-12) {
        w = 1.0; x = 0.0; y = 0.0; z = 0.0;
        return;
    }
    double inv = 1.0 / n;
    w *= inv; x *= inv; y *= inv; z *= inv;
}

static inline void quatRotateWorldFromBody(
    double qw, double qx, double qy, double qz,
    double vx, double vy, double vz,
    double &vpx, double &vpy, double &vpz)
{
    // Optimized: t = 2*cross(q_vec, v); v' = v + qw*t + cross(q_vec, t)
    double tx = 2.0 * (qy * vz - qz * vy);
    double ty = 2.0 * (qz * vx - qx * vz);
    double tz = 2.0 * (qx * vy - qy * vx);

    vpx = vx + qw * tx + (qy * tz - qz * ty);
    vpy = vy + qw * ty + (qz * tx - qx * tz);
    vpz = vz + qw * tz + (qx * ty - qy * tx);
}

struct AxisState {
    // stillness
    int still_count = 0;

    // 1D accel KF
    double a_hat = 0.0;
    double P_1d  = 1.0;

    // light LPF accel
    bool   has_light_prev = false;
    double a_light_prev   = 0.0;

    // 2-state KF: x = [v, b]
    double v2 = 0.0;
    double b2 = 0.0;

    // covariance
    double P00 = 1.0, P01 = 0.0, P10 = 0.0, P11 = 1.0;
};

class ImuKfLogger {
public:
    ImuKfLogger(ros::NodeHandle &nh)
    {
        // Params
        nh.param<std::string>("topic", topic_, std::string("/imu_data_left"));
        nh.param<std::string>("out_csv", out_csv_, std::string("imu_log_ros.csv"));

        nh.param("deadband_x", deadband_x_, 0.15);
        nh.param("deadband_y", deadband_y_, 0.15);
        nh.param("deadband_z", deadband_z_, 0.30);
        nh.param("zupt_steps", zupt_steps_, 5);

        nh.param("Q", Q_, 0.05);
        nh.param("R", R_, 0.20);

        nh.param("lpf_alpha", lpf_alpha_, 0.3);

        nh.param("q_v", q_v_, 0.5);
        nh.param("q_b", q_b_, 0.01);
        nh.param("r_zupt", r_zupt_, 1e-4);

        nh.param("dt_min", dt_min_, 0.001);
        nh.param("dt_max", dt_max_, 0.050);

        // Open CSV
        file_.open(out_csv_, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open CSV: " + out_csv_);
        }

        file_ << "time_ms,"
              << "ax_raw_mps2,ay_raw_mps2,az_raw_mps2,"
              << "ax_kf_mps2,ay_kf_mps2,az_kf_mps2,"
              << "vx_2state_kf_lightlpf_mps,vy_2state_kf_lightlpf_mps,vz_2state_kf_lightlpf_mps,"
              << "vx_world_from_kf_mps,vy_world_from_kf_mps,vz_world_from_kf_mps,"
              << "v_mag\n";

        sub_ = nh.subscribe(topic_, 50, &ImuKfLogger::cb, this);

        ROS_INFO_STREAM("Subscribed to " << topic_ << ", logging to " << out_csv_);
    }

    ~ImuKfLogger()
    {
        if (file_.is_open()) file_.close();
    }

private:
    void cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        const auto &d = msg->data;

        bool have_quat = false;

        double t_ms = 0.0;
        double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
        double ax = 0.0, ay = 0.0, az = 0.0;

        if (d.size() == 4) {
            // [t, ax, ay, az]
            t_ms = d[0];
            ax = d[1]; ay = d[2]; az = d[3];
        } else if (d.size() == 8) {
            // [t, qw, qx, qy, qz, ax, ay, az]
            t_ms = d[0];
            qw = d[1]; qx = d[2]; qy = d[3]; qz = d[4];
            ax = d[5]; ay = d[6]; az = d[7];
            have_quat = true;
        } else {
            // Unknown packet
            return;
        }

        if (!have_prev_time_) {
            t_prev_ms_ = t_ms;
            have_prev_time_ = true;
            return;
        }

        double dt = (t_ms - t_prev_ms_) * 1e-3;
        t_prev_ms_ = t_ms;

        if (dt < dt_min_ || dt > dt_max_) {
            return;
        }

        if (have_quat) {
            quatNormalize(qw, qx, qy, qz);
        }

        // Shared F
        const double F00 = 1.0, F01 = -dt;
        const double F10 = 0.0, F11 = 1.0;

        // Process axes
        double a_raw_x = ax, a_raw_y = ay, a_raw_z = az;

        double a_kf_x = processAxis("x", ax, deadband_x_, F00, F01, F10, F11, dt);
        double a_kf_y = processAxis("y", ay, deadband_y_, F00, F01, F10, F11, dt);
        double a_kf_z = processAxis("z", az, deadband_z_, F00, F01, F10, F11, dt);

        // Body velocities from 2-state KF
        const double vx_b = x_.v2;
        const double vy_b = y_.v2;
        const double vz_b = z_.v2;

        // World rotation if quat exists
        double vx_w = std::numeric_limits<double>::quiet_NaN();
        double vy_w = std::numeric_limits<double>::quiet_NaN();
        double vz_w = std::numeric_limits<double>::quiet_NaN();
        double v_mag = std::numeric_limits<double>::quiet_NaN();

        if (have_quat) {
            quatRotateWorldFromBody(qw, qx, qy, qz, vx_b, vy_b, vz_b, vx_w, vy_w, vz_w);
            v_mag = std::sqrt(vx_w*vx_w + vy_w*vy_w + vz_w*vz_w);
        }

        file_ << std::fixed << std::setprecision(6)
              << t_ms << ","
              << a_raw_x << "," << a_raw_y << "," << a_raw_z << ","
              << a_kf_x  << "," << a_kf_y  << "," << a_kf_z  << ","
              << vx_b    << "," << vy_b    << "," << vz_b    << ","
              << vx_w    << "," << vy_w    << "," << vz_w    << ","
              << v_mag << "\n";

        file_.flush();
    }

    double processAxis(const std::string &axis, double a_in, double deadband,
                       double F00, double F01, double F10, double F11, double dt)
    {
        AxisState *S = nullptr;
        if (axis == "x") S = &x_;
        else if (axis == "y") S = &y_;
        else S = &z_;

        // 1D accel KF (with deadbanded measurement)
        double z = a_in;
        if (std::abs(z) < deadband) z = 0.0;

        S->P_1d = S->P_1d + Q_;
        double K = S->P_1d / (S->P_1d + R_);
        S->a_hat = S->a_hat + K * (z - S->a_hat);
        S->P_1d  = (1.0 - K) * S->P_1d;

        // light LPF for u_lpf (no deadband clamp)
        double a_light = 0.0;
        if (!S->has_light_prev) {
            a_light = a_in;
            S->has_light_prev = true;
        } else {
            a_light = lpf_alpha_ * S->a_light_prev + (1.0 - lpf_alpha_) * a_in;
        }
        S->a_light_prev = a_light;
        const double u_lpf = a_light;

        // stillness counter uses u_lpf vs deadband
        if (std::abs(u_lpf) <= deadband) S->still_count += 1;
        else S->still_count = 0;

        const bool still = (S->still_count >= zupt_steps_);

        // 2-state KF predict: v = v + (u - b)*dt
        S->v2 = S->v2 + (u_lpf - S->b2) * dt;

        // Predict covariance: P = F P F^T + Q
        double A00 = F00 * S->P00 + F01 * S->P10;
        double A01 = F00 * S->P01 + F01 * S->P11;
        double A10 = F10 * S->P00 + F11 * S->P10;
        double A11 = F10 * S->P01 + F11 * S->P11;

        double P00p = A00 * F00 + A01 * F01;
        double P01p = A00 * F10 + A01 * F11;
        double P10p = A10 * F00 + A11 * F01;
        double P11p = A10 * F10 + A11 * F11;

        P00p += q_v_ * dt * dt;
        P11p += q_b_ * dt;

        S->P00 = P00p; S->P01 = P01p; S->P10 = P10p; S->P11 = P11p;

        // ZUPT update for this axis only
        if (still) {
            double y = -S->v2;          // measurement v=0
            double Sm = S->P00 + r_zupt_;
            double K0 = S->P00 / Sm;
            double K1 = S->P10 / Sm;

            S->v2 = S->v2 + K0 * y;
            S->b2 = S->b2 + K1 * y;

            double P00_new = (1.0 - K0) * S->P00;
            double P01_new = (1.0 - K0) * S->P01;
            double P10_new = S->P10 - K1 * S->P00;
            double P11_new = S->P11 - K1 * S->P01;

            S->P00 = P00_new; S->P01 = P01_new; S->P10 = P10_new; S->P11 = P11_new;
        }

        return S->a_hat;
    }

private:
    ros::Subscriber sub_;
    std::ofstream file_;

    std::string topic_;
    std::string out_csv_;

    double deadband_x_, deadband_y_, deadband_z_;
    int zupt_steps_;

    double Q_, R_;
    double lpf_alpha_;
    double q_v_, q_b_, r_zupt_;
    double dt_min_, dt_max_;

    bool have_prev_time_ = false;
    double t_prev_ms_ = 0.0;

    AxisState x_, y_, z_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_kf_logger");
    ros::NodeHandle nh("~");

    try {
        ImuKfLogger logger(nh);
        ros::spin();
    } catch (const std::exception &e) {
        ROS_ERROR("%s", e.what());
        return 1;
    }
    return 0;
}
