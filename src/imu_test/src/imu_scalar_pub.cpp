#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

// Publishers for gyro
ros::Publisher gyro_x_pub;
ros::Publisher gyro_y_pub;
ros::Publisher gyro_z_pub;

// Publishers for accel
ros::Publisher accel_x_pub;
ros::Publisher accel_y_pub;
ros::Publisher accel_z_pub;

// (Optional) publishers for orientation as quaternion
ros::Publisher orient_w_pub;
ros::Publisher orient_x_pub;
ros::Publisher orient_y_pub;
ros::Publisher orient_z_pub;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // --- Gyro ---
  std_msgs::Float64 gx; gx.data = msg->angular_velocity.x;
  std_msgs::Float64 gy; gy.data = msg->angular_velocity.y;
  std_msgs::Float64 gz; gz.data = msg->angular_velocity.z;
  gyro_x_pub.publish(gx);
  gyro_y_pub.publish(gy);
  gyro_z_pub.publish(gz);

  // --- Accel ---
  std_msgs::Float64 ax; ax.data = msg->linear_acceleration.x;
  std_msgs::Float64 ay; ay.data = msg->linear_acceleration.y;
  std_msgs::Float64 az; az.data = msg->linear_acceleration.z;
  accel_x_pub.publish(ax);
  accel_y_pub.publish(ay);
  accel_z_pub.publish(az);

  // --- Orientation (quaternion) ---
  std_msgs::Float64 ow; ow.data = msg->orientation.w;
  std_msgs::Float64 ox; ox.data = msg->orientation.x;
  std_msgs::Float64 oy; oy.data = msg->orientation.y;
  std_msgs::Float64 oz; oz.data = msg->orientation.z;
  orient_w_pub.publish(ow);
  orient_x_pub.publish(ox);
  orient_y_pub.publish(oy);
  orient_z_pub.publish(oz);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle nh;

  // Subscribe to IMU topic
  ros::Subscriber sub = nh.subscribe("imu", 10, imuCallback);

  // Advertise scalar topics
  gyro_x_pub   = nh.advertise<std_msgs::Float64>("imu/gyro_x",   10);
  gyro_y_pub   = nh.advertise<std_msgs::Float64>("imu/gyro_y",   10);
  gyro_z_pub   = nh.advertise<std_msgs::Float64>("imu/gyro_z",   10);

  accel_x_pub  = nh.advertise<std_msgs::Float64>("imu/accel_x",  10);
  accel_y_pub  = nh.advertise<std_msgs::Float64>("imu/accel_y",  10);
  accel_z_pub  = nh.advertise<std_msgs::Float64>("imu/accel_z",  10);

  orient_w_pub = nh.advertise<std_msgs::Float64>("imu/orient_w", 10);
  orient_x_pub = nh.advertise<std_msgs::Float64>("imu/orient_x", 10);
  orient_y_pub = nh.advertise<std_msgs::Float64>("imu/orient_y", 10);
  orient_z_pub = nh.advertise<std_msgs::Float64>("imu/orient_z", 10);

  ROS_INFO("IMU scalar publisher started; plotting on rqt_plot is now easy.");

  ros::spin();
  return 0;
}
