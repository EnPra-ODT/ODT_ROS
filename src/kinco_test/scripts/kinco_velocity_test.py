#!/usr/bin/env python3
import rospy
import can
import struct
from std_msgs.msg import Float64

# === USER SETTINGS ===
NODE_ID = 1
CAN_IFACE = "can0"
ENCODER_RES = 65536  # from datasheet (internal resolution)
MAX_RPM = 5000       # default max speed limit (0x607F)


class KincoCanopenDriver:
    def __init__(self):
        # Open SocketCAN bus
        self.bus = can.interface.Bus(channel=CAN_IFACE, bustype="socketcan")

        self.pub_actual_vel = rospy.Publisher("actual_velocity_rpm", Float64, queue_size=10)
        rospy.Subscriber("target_velocity_rpm", Float64, self.target_velocity_cb)
        rospy.loginfo("Initializing Kinco servo via SDO...")
        self.init_drive()

        # Start timer to poll actual velocity via SDO (0x606C)
        self.poll_period = 0.05  # 50 ms = 20 Hz
        rospy.Timer(rospy.Duration(self.poll_period), self.poll_actual_velocity)

    # ----------------- SDO helper -----------------
    def send_sdo(self, cs, index, subindex, data_bytes):
        """
        cs: command specifier (e.g. 0x2F, 0x2B, 0x23, 0x40)
        index: 16-bit (e.g. 0x6060)
        subindex: 8-bit
        data_bytes: list of up to 4 bytes (LSB first)
        """
        cob_id = 0x600 + NODE_ID
        idx_lo = index & 0xFF
        idx_hi = (index >> 8) & 0xFF

        data = [0] * 8
        data[0] = cs
        data[1] = idx_lo
        data[2] = idx_hi
        data[3] = subindex

        for i, b in enumerate(data_bytes):
            if i >= 4:
                break
            data[4 + i] = b

        msg = can.Message(arbitration_id=cob_id,data=data,is_extended_id=False)
        self.bus.send(msg)

    # ----------------- Init sequence -----------------
    def init_drive(self):
        # 0) (optional but common) Shutdown (0x0006)
        self.send_sdo(0x2B, 0x6040, 0x00, [0x06, 0x00])
        rospy.sleep(0.05)

        # 1) Switch on (0x0007)
        # 601h 2B 40 60 00 07 00 00 00
        self.send_sdo(0x2B, 0x6040, 0x00, [0x07, 0x00])
        rospy.sleep(0.05)

        # 2) Set mode of operation = 3 (Profile Velocity)
        # 601h 2F 60 60 00 03 00 00 00
        self.send_sdo(0x2F, 0x6060, 0x00, [0x03])
        rospy.sleep(0.05)

        # 3) Set target velocity (0x60FF) to 0 (int32)
        # 601h 23 FF 60 00 00 00 00 00
        self.send_sdo(0x23, 0x60FF, 0x00, [0x00, 0x00, 0x00, 0x00])
        rospy.sleep(0.05)

        # 4) Enable operation (0x000F)
        self.send_sdo(0x2B, 0x6040, 0x00, [0x0F, 0x00])
        rospy.sleep(0.05)

        rospy.loginfo("Drive set to Profile Velocity, target=0, then enabled.")


    # ----------------- Target velocity callback -----------------
    def target_velocity_cb(self, msg):
        rpm = msg.data

        # clamp to safe limit
        if rpm > MAX_RPM:
            rpm = MAX_RPM
        if rpm < -MAX_RPM:
            rpm = -MAX_RPM

        # Convert rpm -> DEC using datasheet formula:
        # DEC = (rpm * 512 * encoder_resolution) / 1875
        dec = int(round(rpm * 512.0 * ENCODER_RES / 1875.0))

        # 32-bit signed => little endian
        dec_bytes = list(struct.pack("<i", dec))  # 4 bytes LSB first

        # Write Target Velocity via SDO: index 0x60FF, sub 0, int32
        self.send_sdo(0x23, 0x60FF, 0x00, dec_bytes)

        rospy.loginfo("Sent target velocity: %.1f rpm (DEC=%d)" % (rpm, dec))

    # ----------------- Poll actual velocity via SDO -----------------
    def poll_actual_velocity(self, event):
        """
        Periodically send SDO upload request for Actual Velocity (0x606C)
        and publish result as /actual_velocity_rpm
        """
        # Send SDO upload request
        self.send_sdo(0x40, 0x606C, 0x00, [])

        reply_cob_id = 0x580 + NODE_ID
        dec_val = None

        # Try for a short time to get the matching reply
        end_time = rospy.Time.now() + rospy.Duration(0.02)  # 20 ms window
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            msg = self.bus.recv(timeout=0.005)
            if msg is None:
                continue

            if msg.arbitration_id != reply_cob_id:
                # Not our SDO reply, ignore (could be other traffic)
                continue

            data = msg.data
            # Check index echoed back matches 0x606C
            if len(data) >= 8 and data[1] == 0x6C and data[2] == 0x60:
                # bytes 4–7 = int32 DEC little endian
                dec_bytes = bytes(data[4:8])
                dec_val = struct.unpack("<i", dec_bytes)[0]
                break

        if dec_val is None:
            # No valid response this cycle
            return

        # Convert DEC back to rpm:
        # rpm = DEC * 1875 / (512 * encoder_resolution)
        rpm = dec_val * 1875.0 / (512.0 * ENCODER_RES)
        self.pub_actual_vel.publish(rpm)


def main():
    rospy.init_node("kinco_canopen_velocity_test")
    driver = KincoCanopenDriver()
    rospy.loginfo("Kinco CANopen velocity test node (SDO-based feedback) started.")
    rospy.spin()


if __name__ == "__main__":
    main()

