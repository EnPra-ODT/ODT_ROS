#!/usr/bin/env python3
import rospy
import csv
import os
from std_msgs.msg import Float64, Float64MultiArray
import datetime

class VmagContactLogger:
    def __init__(self):
        rospy.init_node("vmag_contact_logger")

        # ---- params ----
        self.out_dir  = rospy.get_param("~out_dir", ".")
        default_name = datetime.datetime.now().strftime(
            "vmag_contact_%Y-%m-%d_%H-%M-%S.csv"
        )

        self.filename = rospy.get_param("~filename", default_name)

        os.makedirs(self.out_dir, exist_ok=True)
        self.filepath = os.path.join(self.out_dir, self.filename)

        # ---- state ----
        self.v_mag_active = None
        self.contact_pair = None

        # ---- CSV ----
        self.csv_file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "time_sec",
            "v_mag_active",
            "pair1",
            "pair2",
            "pair3",
            "pair4"
        ])
        self.csv_file.flush()

        # ---- subs ----
        rospy.Subscriber(
            "/v_mag_active",
            Float64,
            self.vmag_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/foot_contact_pair",
            Float64MultiArray,
            self.contact_cb,
            queue_size=10
        )

        rospy.loginfo(f"Logging to {self.filepath}")

    def vmag_cb(self, msg):
        self.v_mag_active = msg.data
        self.try_write()

    def contact_cb(self, msg):
        if len(msg.data) < 4:
            return
        self.contact_pair = msg.data[:4]
        self.try_write()

    def try_write(self):
        if self.v_mag_active is None or self.contact_pair is None:
            return

        t = rospy.Time.now().to_sec()
        self.writer.writerow([
            f"{t:.6f}",
            f"{self.v_mag_active:.6f}",
            int(self.contact_pair[0]),
            int(self.contact_pair[1]),
            int(self.contact_pair[2]),
            int(self.contact_pair[3]),
        ])
        self.csv_file.flush()

    def shutdown(self):
        rospy.loginfo("Closing CSV file")
        self.csv_file.close()

if __name__ == "__main__":
    logger = VmagContactLogger()
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()
