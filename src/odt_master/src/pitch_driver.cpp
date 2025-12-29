#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

static const int NODE_ID = 1;
static const char *CAN_IFACE = "can0";

static const int ENCODER_RES = 65536;
static const int MAX_RPM = 5000;

class KincoCanopenDriver{
public:
  KincoCanopenDriver(ros::NodeHandle& nh){
    sock_ = -1;
    openCanSocket(CAN_IFACE);

    pub_actual_vel_ = nh.advertise<std_msgs::Float64>("actual_velocity_rpm", 10);
    sub_target_vel_ = nh.subscribe("target_velocity_rpm", 10, &KincoCanopenDriver::targetVelocityCb, this);

    ROS_INFO("Initializing Kinco servo via SDO...");
    initDrive();

    poll_period_ = 0.05;
    timer_ = nh.createTimer(ros::Duration(poll_period_), &KincoCanopenDriver::pollActualVelocity, this);
  }

  ~KincoCanopenDriver(){
    if (sock_ >= 0){close(sock_);}
    }

private:
  int sock_;
  ros::Publisher pub_actual_vel_;
  ros::Subscriber sub_target_vel_;
  ros::Timer timer_;
  double poll_period_;

  void openCanSocket(const char *ifname){
    struct ifreq ifr;
    struct sockaddr_can addr;

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0){throw std::runtime_error("socket(PF_CAN) failed");}

    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname);

    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0){throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");}

    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0){throw std::runtime_error("bind(AF_CAN) failed");}

    ROS_INFO("SocketCAN bound to interface: %s", ifname);
  }

  bool sendFrame(uint32_t cob_id, const uint8_t data[8]){
    struct can_frame frame;

    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = cob_id;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    int n = write(sock_, &frame, sizeof(frame));
    if (n != (int)sizeof(frame)) {
      ROS_WARN("CAN write failed (n=%d errno=%d)", n, errno);
      return false;
    }
    return true;
  }

  bool recvFrame(struct can_frame *out, int timeout_ms){
    fd_set rfds;
    struct timeval tv;
    int ret;

    FD_ZERO(&rfds);
    FD_SET(sock_, &rfds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(sock_ + 1, &rfds, NULL, NULL, &tv);
    if (ret <= 0)return false;

    int n = read(sock_, out, sizeof(*out));
    if (n != (int)sizeof(*out)) return false;

    return true;
  }

  void sendSdo(uint8_t cs, uint16_t index, uint8_t subindex, const uint8_t *data_bytes, int data_len){
    uint32_t cob_id = 0x600 + NODE_ID;
    uint8_t data[8];

    std::memset(data, 0, sizeof(data));
    data[0] = cs;
    data[1] = (uint8_t)(index & 0xFF);
    data[2] = (uint8_t)((index >> 8) & 0xFF);
    data[3] = subindex;

    for (int i = 0; i < data_len && i < 4; i++)data[4 + i] = data_bytes[i];
    sendFrame(cob_id, data);
  }

  void sdoWriteU16(uint16_t index, uint8_t sub, uint16_t val){
    uint8_t b[2];

    b[0] = (uint8_t)(val & 0xFF);
    b[1] = (uint8_t)((val >> 8) & 0xFF);

    sendSdo(0x2B, index, sub, b, 2);
  }

  void sdoWriteI8(uint16_t index, uint8_t sub, int8_t val){
    uint8_t b[1];

    b[0] = (uint8_t)val;
    sendSdo(0x2F, index, sub, b, 1);
  }

  void sdoWriteI32(uint16_t index, uint8_t sub, int32_t val){
    uint8_t b[4];

    b[0] = (uint8_t)(val & 0xFF);
    b[1] = (uint8_t)((val >> 8) & 0xFF);
    b[2] = (uint8_t)((val >> 16) & 0xFF);
    b[3] = (uint8_t)((val >> 24) & 0xFF);

    sendSdo(0x23, index, sub, b, 4);
  }

  bool sdoReadI32(uint16_t index, uint8_t sub, int32_t *out_val, int window_ms){
    uint32_t req_cob = 0x600 + NODE_ID;
    uint32_t rep_cob = 0x580 + NODE_ID;

    (void)req_cob;

    sendSdo(0x40, index, sub, NULL, 0);

    ros::Time end = ros::Time::now() + ros::Duration(window_ms / 1000.0);
    while (ros::Time::now() < end && ros::ok()) {
      struct can_frame f;
      if (!recvFrame(&f, 5))continue;
      if ((f.can_id & CAN_EFF_MASK) != rep_cob)continue;
      if (f.can_dlc < 8)continue;
      if (f.data[1] != (uint8_t)(index & 0xFF))continue;
      if (f.data[2] != (uint8_t)((index >> 8) & 0xFF))continue;
      if (f.data[3] != sub) continue;

      int32_t v = 0;
      v |= ((int32_t)f.data[4]) << 0;
      v |= ((int32_t)f.data[5]) << 8;
      v |= ((int32_t)f.data[6]) << 16;
      v |= ((int32_t)f.data[7]) << 24;

      *out_val = v;
      return true;
    }
    return false;
  }

  void initDrive(void){
    /*
      * Requested order (plus optional Shutdown first):
      * 6040=0006 (optional)
      * 6040=0007 (Switch on)
      * 6060=3    (Profile Velocity)
      * 60FF=0    (Goal velocity = 0)
      * 6040=000F (Enable operation)
      */

    sdoWriteU16(0x6040, 0x00, 0x0006);
    ros::Duration(0.05).sleep();

    sdoWriteU16(0x6040, 0x00, 0x0007);
    ros::Duration(0.05).sleep();

    sdoWriteI8(0x6060, 0x00, 3);
    ros::Duration(0.05).sleep();

    sdoWriteI32(0x60FF, 0x00, 0);
    ros::Duration(0.05).sleep();

    sdoWriteU16(0x6040, 0x00, 0x000F);
    ros::Duration(0.05).sleep();

    ROS_INFO("Drive should now be in Profile Velocity, target=0, enabled.");
  }

  void targetVelocityCb(const std_msgs::Float64::ConstPtr& msg){
    double rpm = msg->data;

    if (rpm > MAX_RPM) rpm = MAX_RPM;
    if (rpm < -MAX_RPM)rpm = -MAX_RPM;

    double dec_f = rpm * 512.0 * (double)ENCODER_RES / 1875.0;
    int32_t dec = (int32_t)llround(dec_f);

    sdoWriteI32(0x60FF, 0x00, dec);

    ROS_INFO("Sent target velocity: %.1f rpm (DEC=%d)", rpm, (int)dec);
  }

  void pollActualVelocity(const ros::TimerEvent&){
    int32_t dec_val;
    if (!sdoReadI32(0x606C, 0x00, &dec_val, 20))return;

    double rpm = (double)dec_val * 1875.0 / (512.0 * (double)ENCODER_RES);

    std_msgs::Float64 out;
    out.data = rpm;
    pub_actual_vel_.publish(out);
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "kinco_canopen_velocity_test");
  ros::NodeHandle nh;

  try {
      KincoCanopenDriver driver(nh);
      ROS_INFO("Kinco CANopen velocity test node (SDO-based feedback) started.");
      ros::spin();
  } catch (const std::exception& e) {
      ROS_ERROR("Fatal: %s", e.what());
      return 1;
  }

  return 0;
}
