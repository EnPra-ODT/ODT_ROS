#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

// --- Parameters ---
const double MAX_VEL_LIMIT = 180.0;
const double YAW_SCALE_DEG = 2.0;   // joystick full-scale → ±30 deg (adjust)

// Latest joystick values
double g_axes_1 = 0.0;
double g_axes_4 = 0.0;

double g_cumulative_angle = 0.0;

// Only update joystick state
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joy->axes.size() > 4) {
        g_axes_1 = joy->axes[1];   // forward
        g_axes_4 = joy->axes[4];   // yaw
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joystick_values_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("joy", 10, joyCallback);

    ros::Publisher vmag_pub = nh.advertise<std_msgs::Float64>("/v_mag_active", 10);
    ros::Publisher imu_right_pub = nh.advertise<std_msgs::Float64MultiArray>("/imu_data_right", 10);

    ros::Rate loop_rate(10);  // 10 Hz

    ROS_INFO("Joystick → v_mag_active + imu_data_right started.");

    while (ros::ok()){
        // ---- velocity ----
        std_msgs::Float64 v_msg;
        if (g_axes_1 > 0.0)
            v_msg.data = g_axes_1 * MAX_VEL_LIMIT;
        else
            v_msg.data = 0.0;

        vmag_pub.publish(v_msg);

        
        std_msgs::Float64MultiArray imu_msg;
        imu_msg.data.resize(7);

        g_cumulative_angle += g_axes_4 * YAW_SCALE_DEG;

        imu_msg.data[0] = 0.0;                         // t_ms (unused)
        imu_msg.data[1] = 0.0;                         // ax
        imu_msg.data[2] = 0.0;                         // ay
        imu_msg.data[3] = 0.0;                         // az
        imu_msg.data[4] = g_cumulative_angle;   // yaw (deg) ← IMPORTANT
        imu_msg.data[5] = 0.0;                         // pitch
        imu_msg.data[6] = 0.0;                         // roll

        imu_right_pub.publish(imu_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
