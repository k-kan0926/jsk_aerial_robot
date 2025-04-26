#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

class DS4MpaController:
    def __init__(self):
        rospy.init_node("ds4_mpa_controller")

        self.v1_mpa = 0.2
        self.v2_mpa = 0.2
        self.step = 0.002
        self.v1_min, self.v1_max = 0, 0.7
        self.v2_min, self.v2_max = 0, 0.7

        self.l1_pressed = False
        self.mpa_pub = rospy.Publisher("/mpa_cmd", Vector3, queue_size=1)
        rospy.Subscriber("/kinikun/joy", Joy, self.joy_callback)
        rospy.loginfo("DS4 MPA Controller Ready")
        rospy.spin()

    def joy_callback(self, msg):
        l1 = msg.buttons[4]
        axis_h = msg.axes[9]  # ←→
        axis_v = msg.axes[10]  # ↑↓
        circle = msg.buttons[2]


        if l1:
            updated = False
            if axis_v == 1:
                self.v1_mpa = min(self.v1_mpa + self.step, self.v1_max)
                updated = True
            elif axis_v == -1:
                self.v1_mpa = max(self.v1_mpa - self.step, self.v1_min)
                updated = True
            if axis_h == 1:
                self.v2_mpa = min(self.v2_mpa + self.step, self.v2_max)
                updated = True
            elif axis_h == -1:
                self.v2_mpa = max(self.v2_mpa - self.step, self.v2_min)
                updated = True
            if circle == 1:
                self.v1_mpa = 0.2
                self.v2_mpa = 0.2
                updated = True

             # Publish only if values are updated

            if updated:
                v1 = self.v1_mpa * 4096 / 0.9
                v2 = self.v2_mpa * 4096 / 0.9
                self.mpa_pub.publish(Vector3(v1, v2, 0))
                rospy.loginfo(f"v1={self.v1_mpa:.2f}, v2={self.v2_mpa:.2f}")

if __name__ == "__main__":
    try:
        DS4MpaController()
    except rospy.ROSInterruptException:
        pass
