#!/usr/bin/env python3
import rospy
import serial
import sys
import select
from std_msgs.msg import Float32, Float64MultiArray

def main():
    isDebug = False

    rospy.init_node('loadcell_reader')

    start_time = rospy.Time.now()
    OFFSET_DELAY = rospy.Duration(10.0)  # seconds

    # --- publishers ---
    pub_ch1 = rospy.Publisher('/loadcell_ch1', Float32, queue_size=10)
    pub_ch2 = rospy.Publisher('/loadcell_ch2', Float32, queue_size=10)
    pub_ch3 = rospy.Publisher('/loadcell_ch3', Float32, queue_size=10)
    pub_ch4 = rospy.Publisher('/loadcell_ch4', Float32, queue_size=10)
    pub_ch5 = rospy.Publisher('/loadcell_ch5', Float32, queue_size=10)
    pub_ch6 = rospy.Publisher('/loadcell_ch6', Float32, queue_size=10)
    pub_ch7 = rospy.Publisher('/loadcell_ch7', Float32, queue_size=10)
    pub_ch8 = rospy.Publisher('/loadcell_ch8', Float32, queue_size=10)

    pub_contact_pair = rospy.Publisher('/foot_contact_pair', Float64MultiArray, queue_size=10)
    pub_contact_array = rospy.Publisher('/foot_contact_array', Float64MultiArray, queue_size=10)

    port = rospy.get_param('~port', '/dev/ttyACM0')
    baud = rospy.get_param('~baud', 115200)
    threshold = rospy.get_param('~threshold', 15.0)

    ser = serial.Serial(port, baud, timeout=1)
    rospy.sleep(2.0)

    rospy.loginfo("Node started. Press 'r' to set offset.")

    # --- offset values ---
    offset_ch1 = 0.0
    offset_ch2 = 0.0
    offset_ch3 = 0.0
    offset_ch4 = 0.0
    offset_ch5 = 0.0
    offset_ch6 = 0.0
    offset_ch7 = 0.0
    offset_ch8 = 0.0

    offset_set = False

    # last raw samples (used for offset)
    last_raw_ch1 = 0.0
    last_raw_ch2 = 0.0
    last_raw_ch3 = 0.0
    last_raw_ch4 = 0.0
    last_raw_ch5 = 0.0
    last_raw_ch6 = 0.0
    last_raw_ch7 = 0.0
    last_raw_ch8 = 0.0

    have_valid_sample = False

    while not rospy.is_shutdown():
        if isDebug:
            rospy.loginfo_throttle(1.0, "loop alive")

        # --- key input (non-blocking) ---
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'r':
                if have_valid_sample:
                    offset_ch1 = last_raw_ch1
                    offset_ch2 = last_raw_ch2
                    offset_ch3 = last_raw_ch3
                    offset_ch4 = last_raw_ch4
                    offset_ch5 = last_raw_ch5
                    offset_ch6 = last_raw_ch6
                    offset_ch7 = last_raw_ch7
                    offset_ch8 = last_raw_ch8
                    offset_set = True
                    rospy.loginfo("Offset set (manual)")
                else:
                    rospy.logwarn("Offset not set: no valid serial sample received yet.")

        # --- serial read ---
        line = ser.readline()
        if not line:
            # keep looping; auto-offset may still happen only after valid sample
            continue

        try:
            line = line.decode(errors='ignore').strip()
            vals = line.split(',')

            if len(vals) != 8:
                rospy.logwarn_throttle(1.0, f"Invalid line: {line}")
                continue

            raw_ch1 = float(vals[0])
            raw_ch2 = float(vals[1])
            raw_ch3 = float(vals[2])
            raw_ch4 = float(vals[3])
            raw_ch5 = float(vals[4])
            raw_ch6 = float(vals[5])
            raw_ch7 = float(vals[6])
            raw_ch8 = float(vals[7])

            # update last raw for offset
            last_raw_ch1 = raw_ch1
            last_raw_ch2 = raw_ch2
            last_raw_ch3 = raw_ch3
            last_raw_ch4 = raw_ch4
            last_raw_ch5 = raw_ch5
            last_raw_ch6 = raw_ch6
            last_raw_ch7 = raw_ch7
            last_raw_ch8 = raw_ch8
            have_valid_sample = True

            # --- auto offset after delay (only if we have at least one valid sample) ---
            now = rospy.Time.now()
            if (not offset_set) and have_valid_sample and (now - start_time) > OFFSET_DELAY:
                offset_ch1 = last_raw_ch1
                offset_ch2 = last_raw_ch2
                offset_ch3 = last_raw_ch3
                offset_ch4 = last_raw_ch4
                offset_ch5 = last_raw_ch5
                offset_ch6 = last_raw_ch6
                offset_ch7 = last_raw_ch7
                offset_ch8 = last_raw_ch8
                offset_set = True
                rospy.loginfo("Offset automatically set after 10 seconds")

            # --- don't publish while in offset phase ---
            if not offset_set:
                continue

            # apply offsets
            raw_ch1 -= offset_ch1
            raw_ch2 -= offset_ch2
            raw_ch3 -= offset_ch3
            raw_ch4 -= offset_ch4
            raw_ch5 -= offset_ch5
            raw_ch6 -= offset_ch6
            raw_ch7 -= offset_ch7
            raw_ch8 -= offset_ch8

            # convert to N (your scale)
            ch1_N = abs(raw_ch1 * 0.00981)
            ch2_N = abs(raw_ch2 * 0.00981)
            ch3_N = abs(raw_ch3 * 0.00981)
            ch4_N = abs(raw_ch4 * 0.00981)
            ch5_N = abs(raw_ch5 * 0.00981)
            ch6_N = abs(raw_ch6 * 0.00981)
            ch7_N = abs(raw_ch7 * 0.00981)
            ch8_N = abs(raw_ch8 * 0.00981)

            # publish channels
            pub_ch1.publish(ch1_N)
            pub_ch2.publish(ch2_N)
            pub_ch3.publish(ch3_N)
            pub_ch4.publish(ch4_N)
            pub_ch5.publish(ch5_N)
            pub_ch6.publish(ch6_N)
            pub_ch7.publish(ch7_N)
            pub_ch8.publish(ch8_N)

            # contact flags
            c1 = ch1_N > threshold
            c2 = ch2_N > threshold
            c3 = ch3_N > threshold
            c4 = ch4_N > threshold
            c5 = ch5_N > threshold
            c6 = ch6_N > threshold
            c7 = ch7_N > threshold
            c8 = ch8_N > threshold

            # per-channel array
            contact_array = Float64MultiArray()
            contact_array.data = [
                1.0 if c1 else 0.0,
                1.0 if c2 else 0.0,
                1.0 if c3 else 0.0,
                1.0 if c4 else 0.0,
                1.0 if c5 else 0.0,
                1.0 if c6 else 0.0,
                1.0 if c7 else 0.0,
                1.0 if c8 else 0.0,
            ]
            pub_contact_array.publish(contact_array)

            # pair contacts
            pair1 = c1 or c2
            pair2 = c3 or c4
            pair3 = c5 or c6
            pair4 = c7 or c8

            pair_array = Float64MultiArray()
            pair_array.data = [
                1.0 if pair1 else 0.0,
                1.0 if pair2 else 0.0,
                1.0 if pair3 else 0.0,
                1.0 if pair4 else 0.0,
            ]
            pub_contact_pair.publish(pair_array)

            if isDebug:
                rospy.loginfo(
                    f"CH1={ch1_N:.2f}N CH2={ch2_N:.2f}N CH3={ch3_N:.2f}N CH4={ch4_N:.2f}N "
                    f"CH5={ch5_N:.2f}N CH6={ch6_N:.2f}N CH7={ch7_N:.2f}N CH8={ch8_N:.2f}N "
                    f"pair={pair_array.data}"
                )
            else:
                rospy.loginfo_throttle(0.2, f"pair = {pair_array.data}")

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    import termios, tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        main()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
