// File: kinco_pdo_driver_with_loadcell_stop_kr.cpp
//
// Procedural (no class) K&R-style ROS node.
//
// Subscribes:
//   /target_velocity_rpm   (std_msgs/Float64)  command from rostopic pub
//   /foot_contact          (std_msgs/Bool)     from loadcell_node.py
//
// Publishes:
//   actual_velocity_rpm    (std_msgs/Float64)
//
// Behavior:
//   If /foot_contact == true, overrides target velocity to stop_rpm (default 0.0).

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <string.h>
#include <stdint.h>
#include <mutex>
#include <string>

#define DEFAULT_NODE_ID        1
#define DEFAULT_CAN_IFACE_NAME "can0"
#define ENCODER_RES            65536
#define DEFAULT_MAX_RPM        5000.0

static ros::Publisher g_pub_actual_vel;
static ros::Subscriber g_sub_target_vel;
static ros::Subscriber g_sub_foot_contact;
static ros::Timer g_recv_timer;
static ros::Timer g_resend_timer;

static int g_can_socket = -1;
static std::string g_can_iface = DEFAULT_CAN_IFACE_NAME;
static int g_node_id = DEFAULT_NODE_ID;

static std::mutex g_mtx;
static bool g_foot_contact = false;
static double g_last_cmd_rpm = 0.0;

static bool g_stop_on_contact = true;
static double g_stop_rpm = 0.0;

static bool g_enable_resend_timer = true;
static double g_resend_hz = 50.0;

static double g_max_rpm = DEFAULT_MAX_RPM;

static bool open_can_socket(void);
static void send_sdo(uint8_t cs, uint16_t idx, uint8_t sub, const uint8_t *data, uint8_t len);
static void configure_pdos(void);
static void init_drive(void);

static int32_t rpm_to_dec(double rpm);
static double dec_to_rpm(int32_t dec);
static double clamp_rpm(double rpm);
static double gated_rpm(double requested_rpm);

static void send_target_velocity_rpm(double rpm);

static void target_velocity_cb(const std_msgs::Float64::ConstPtr& msg);
static void foot_contact_cb(const std_msgs::Bool::ConstPtr& msg);
static void recv_loop(const ros::TimerEvent& ev);
static void resend_loop(const ros::TimerEvent& ev);

static bool open_can_socket(void){
  struct ifreq ifr;
  struct sockaddr_can addr;

  g_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (g_can_socket < 0)
    return false;

  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, g_can_iface.c_str(), IFNAMSIZ - 1);
  if (ioctl(g_can_socket, SIOCGIFINDEX, &ifr) < 0)
    return false;

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(g_can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    return false;

  ROS_INFO("CAN socket ready: %s", g_can_iface.c_str());
  return true;
}

static void send_sdo(uint8_t cs, uint16_t idx, uint8_t sub, const uint8_t *data, uint8_t len){
  struct can_frame f;
  uint8_t i;

  f.can_id = 0x600 + g_node_id;
  f.can_dlc = 8;

  for (i = 0; i < 8; i++)
    f.data[i] = 0;

  f.data[0] = cs;
  f.data[1] = (uint8_t)(idx & 0xFF);
  f.data[2] = (uint8_t)(idx >> 8);
  f.data[3] = sub;

  for (i = 0; i < len && i < 4; i++)
    f.data[4 + i] = data[i];

  (void)write(g_can_socket, &f, sizeof(f));
  usleep(3000);
}

static void configure_pdos(void){
  uint8_t d[4];

  /* ---------- TPDO1: Statusword + Actual Velocity ---------- */

  /* Disable TPDO1 */
  d[0] = 0;
  send_sdo(0x2F, 0x1800, 0x01, d, 1);

  /* Clear TPDO1 mapping */
  d[0] = 0;
  send_sdo(0x2F, 0x1A00, 0x00, d, 1);

  /* Map Statusword 0x6041:00 (16 bits) */
  d[0] = 0x41; d[1] = 0x60; d[2] = 0x00; d[3] = 0x10;
  send_sdo(0x23, 0x1A00, 0x01, d, 4);

  /* Map Actual Velocity 0x606C:00 (32 bits) */
  d[0] = 0x6C; d[1] = 0x60; d[2] = 0x20; d[3] = 0x00;
  send_sdo(0x23, 0x1A00, 0x02, d, 4);

  /* Enable mapping: 2 entries */
  d[0] = 2;
  send_sdo(0x2F, 0x1A00, 0x00, d, 1);

  /* TPDO1 COB-ID = 0x180 + node_id */
  d[0] = (uint8_t)(0x80 + g_node_id);  /* low byte of 0x180+id */
  d[1] = 0x01;                         /* high byte: 0x01 */
  d[2] = 0; d[3] = 0;
  send_sdo(0x23, 0x1800, 0x01, d, 4);

  /* TPDO1 transmission type: 255 (asynchronous) */
  d[0] = 255;
  send_sdo(0x2F, 0x1800, 0x02, d, 1);

  /* ---------- RPDO1: Target Velocity ---------- */

  /* Disable RPDO1 */
  d[0] = 0;
  send_sdo(0x2F, 0x1400, 0x01, d, 1);

  /* Clear RPDO1 mapping */
  d[0] = 0;
  send_sdo(0x2F, 0x1600, 0x00, d, 1);

  /* Map Target Velocity 0x60FF:00 (32 bits) */
  d[0] = 0xFF; d[1] = 0x60; d[2] = 0x20; d[3] = 0x00;
  send_sdo(0x23, 0x1600, 0x01, d, 4);

  /* Enable mapping: 1 entry */
  d[0] = 1;
  send_sdo(0x2F, 0x1600, 0x00, d, 1);

  /* RPDO1 COB-ID = 0x200 + node_id */
  d[0] = (uint8_t)(0x00 + g_node_id);  /* low byte of 0x200+id */
  d[1] = 0x02;                         /* high byte: 0x02 */
  d[2] = 0; d[3] = 0;
  send_sdo(0x23, 0x1400, 0x01, d, 4);

  /* RPDO1 transmission type: 255 (asynchronous) */
  d[0] = 255;
  send_sdo(0x2F, 0x1400, 0x02, d, 1);
}

static void init_drive(void){
  uint8_t d[2];

  /* Profile Velocity mode */
  d[0] = 3;
  send_sdo(0x2F, 0x6060, 0x00, d, 1);

  /* Controlword state machine: shutdown -> switch on -> enable operation */
  d[0] = 0x06; d[1] = 0x00;
  send_sdo(0x2B, 0x6040, 0x00, d, 2);

  d[0] = 0x07; d[1] = 0x00;
  send_sdo(0x2B, 0x6040, 0x00, d, 2);

  d[0] = 0x0F; d[1] = 0x00;
  send_sdo(0x2B, 0x6040, 0x00, d, 2);
}

static int32_t rpm_to_dec(double rpm){
  double dec;

  /* Matches your original mapping */
  dec = rpm * 512.0 * ENCODER_RES / 1875.0;

  if (dec >= 0)
    return (int32_t)(dec + 0.5);
  return (int32_t)(dec - 0.5);
}

static double dec_to_rpm(int32_t dec){
  return (double)dec * 1875.0 / (512.0 * ENCODER_RES);
}

static double clamp_rpm(double rpm){
  if (rpm > g_max_rpm)
    rpm = g_max_rpm;
  if (rpm < -g_max_rpm)
    rpm = -g_max_rpm;
  return rpm;
}

static double gated_rpm(double requested_rpm){
  std::lock_guard<std::mutex> lk(g_mtx);

  if (g_stop_on_contact && g_foot_contact)
    return g_stop_rpm;
  return requested_rpm;
}

static void send_target_velocity_rpm(double rpm){
  struct can_frame f;
  int32_t dec;
  uint8_t i;

  rpm = clamp_rpm(rpm);
  dec = rpm_to_dec(rpm);

  f.can_id = 0x200 + g_node_id;
  f.can_dlc = 8;

  for (i = 0; i < 8; i++)
    f.data[i] = 0;

  f.data[0] = (uint8_t)( dec        & 0xFF);
  f.data[1] = (uint8_t)((dec >> 8)  & 0xFF);
  f.data[2] = (uint8_t)((dec >> 16) & 0xFF);
  f.data[3] = (uint8_t)((dec >> 24) & 0xFF);

  (void)write(g_can_socket, &f, sizeof(f));
}

static void target_velocity_cb(const std_msgs::Float64::ConstPtr& msg){
  double rpm_to_send;

  {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_last_cmd_rpm = msg->data;
  }

  rpm_to_send = gated_rpm(msg->data);
  send_target_velocity_rpm(rpm_to_send);
}

static void foot_contact_cb(const std_msgs::Bool::ConstPtr& msg){
  bool became_true;

  became_true = false;
  {
    std::lock_guard<std::mutex> lk(g_mtx);
    became_true = (!g_foot_contact && msg->data);
    g_foot_contact = msg->data;
  }

  if (g_stop_on_contact && msg->data) {
    send_target_velocity_rpm(g_stop_rpm);
    if (became_true)
      ROS_WARN("Foot contact detected -> forcing STOP (target_velocity_rpm overridden)");
  }
}

static void resend_loop(const ros::TimerEvent& ev){
  (void)ev;

  double last;
  double rpm_to_send;

  {
    std::lock_guard<std::mutex> lk(g_mtx);
    last = g_last_cmd_rpm;
  }

  rpm_to_send = gated_rpm(last);
  send_target_velocity_rpm(rpm_to_send);
}

static void recv_loop(const ros::TimerEvent& ev){
  (void)ev;

  struct can_frame f;
  int n;

  n = read(g_can_socket, &f, sizeof(f));
  if (n <= 0)
    return;

  if (f.can_id == (0x180 + g_node_id) && f.can_dlc >= 6) {
    int32_t dec;
    std_msgs::Float64 msg;

    dec = (int32_t)(
      ((uint32_t)f.data[2]) |
      ((uint32_t)f.data[3] << 8) |
      ((uint32_t)f.data[4] << 16) |
      ((uint32_t)f.data[5] << 24)
    );

    msg.data = dec_to_rpm(dec);
    g_pub_actual_vel.publish(msg);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "pitch_driver");
  ros::NodeHandle nh("~");

  nh.param<std::string>("can_iface", g_can_iface, std::string(DEFAULT_CAN_IFACE_NAME));
  nh.param<int>("node_id", g_node_id, (int)DEFAULT_NODE_ID);
  nh.param<double>("max_rpm", g_max_rpm, (double)DEFAULT_MAX_RPM);
  nh.param<bool>("stop_on_contact", g_stop_on_contact, true);
  nh.param<double>("stop_rpm", g_stop_rpm, 0.0);
  nh.param<bool>("enable_resend_timer", g_enable_resend_timer, true);
  nh.param<double>("resend_hz", g_resend_hz, 50.0);

  g_pub_actual_vel = nh.advertise<std_msgs::Float64>("actual_velocity_rpm", 10);
  g_sub_target_vel = nh.subscribe("/target_velocity_rpm", 10, target_velocity_cb);
  g_sub_foot_contact = nh.subscribe("/foot_contact", 10, foot_contact_cb);

  if (!open_can_socket()) {
    ROS_FATAL("CAN socket open failed");
    return 1;
  }

  configure_pdos();
  init_drive();

  ROS_INFO("Kinco RPDO/TPDO control active (procedural K&R, gated by /foot_contact)");

  g_recv_timer = nh.createTimer(ros::Duration(0.001), recv_loop);

  if (g_enable_resend_timer && g_resend_hz > 0.0)
    g_resend_timer = nh.createTimer(ros::Duration(1.0 / g_resend_hz), resend_loop);

  ros::spin();

  if (g_can_socket >= 0)
    close(g_can_socket);

  return 0;
}
