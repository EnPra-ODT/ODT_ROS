#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

// --- 設定値 ---
const double MAX_VEL_LIMIT = 255.0;
const double ANGLE_SENSITIVITY = 1.0; // ループ周期に合わせて調整してください

// グローバル変数でジョイスティックの最新の状態を保持
double g_axes_1 = 0.0;
double g_axes_4 = 0.0;
double g_cumulative_angle = 0.0;

// コールバックは値の「更新」だけを行う
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    g_axes_1 = joy->axes[1];
    g_axes_4 = joy->axes[4];
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_values_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("joystick_values", 10);
    ros::Subscriber sub = nh.subscribe("joy", 10, joyCallback);

    // 10Hz（0.1秒ごと）でループを実行
    ros::Rate loop_rate(10); 

    ROS_INFO("Joystick Loop Node Started.");

    while (ros::ok()) {
        std_msgs::Float64MultiArray msg;
        msg.data.resize(2);

        // --- 左スティック：速度制御 (0-255) ---
        if (g_axes_1 > 0) {
            msg.data[0] = g_axes_1 * MAX_VEL_LIMIT;
        } else {
            msg.data[0] = 0.0;
        }

        // --- 右スティック：累積角度制御 ---
        // ループごとに現在の傾き（g_axes_4）を加算し続ける
        g_cumulative_angle += g_axes_4 * ANGLE_SENSITIVITY;
        msg.data[1] = g_cumulative_angle;

        pub.publish(msg);

        // コールバック関数の処理（ジョイスティック入力の更新）を実行
        ros::spinOnce();
        
        // 設定した周期（10Hz）になるよう待機
        loop_rate.sleep(); 
    }

    return 0;
}