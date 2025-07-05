#!/usr/bin/env python3
"""
step_param_loader.py（誤差収束監視版）
----------------------------------------
ROSノード：p1 と p2 をステップ状に変化させ、各ステップで
関節角度（arm1_joint）の変化が一定誤差以下で安定するまで待機。
角度が安定したら、次のステップへ移行。各ステップで現在角度をログ出力。
"""
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

FREQUENCY = 40  # [Hz] publish rate
STEP_SIZE = 0.05
MAX_VALUE = 0.75
STABLE_ERROR_THRESHOLD = 0.001  # [rad] 以下の変化を安定とみなす
STABLE_DURATION = 3.0  # [s] 安定判定に必要な持続時間
RESET_HOLD = 5.0  # 最後に5秒静止してリセット

class StepParamLoader:
    def __init__(self):
        rospy.init_node("step_param_loader", anonymous=True)
        self.pub = rospy.Publisher("mpa_cmd", Vector3, queue_size=10)
        rospy.Subscriber("/kinikun1/joint_states", JointState, self.joint_callback)
        self.rate = rospy.Rate(FREQUENCY)

        self.current_angle = 0.0
        self.angle_buffer = []

    def joint_callback(self, msg):
        if "arm1_joint" in msg.name:
            idx = msg.name.index("arm1_joint")
            self.current_angle = msg.position[idx]

    def is_angle_stable(self):
        if len(self.angle_buffer) < int(STABLE_DURATION * FREQUENCY):
            return False
        recent = self.angle_buffer[-int(STABLE_DURATION * FREQUENCY):]
        max_val = max(recent)
        min_val = min(recent)
        return abs(max_val - min_val) < STABLE_ERROR_THRESHOLD

    def publish_step(self, p1, p2):
        v1 = p1 * 4096 / 0.9
        v2 = p2 * 4096 / 0.9
        msg = Vector3(x=v1, y=v2, z=0.0)
        self.pub.publish(msg)
        rospy.set_param("/p1_value", p1)
        rospy.set_param("/p2_value", p2)
        rospy.loginfo(f"Command: p1={p1:.2f}, p2={p2:.2f}")

    def run(self):
        current_p1 = 0.7    
        current_p2 = 0.0

        while not rospy.is_shutdown() and current_p2 <= MAX_VALUE:
            self.publish_step(current_p1, current_p2)
            self.angle_buffer = []

            while not rospy.is_shutdown():
                self.angle_buffer.append(self.current_angle)
                if len(self.angle_buffer) > int(STABLE_DURATION * FREQUENCY) * 2:
                    self.angle_buffer.pop(0)

                if self.is_angle_stable():
                    rospy.loginfo(f"Angle stabilized at {self.current_angle:.4f} rad")
                    break
                self.rate.sleep()

            current_p2 += STEP_SIZE

        rospy.loginfo("All steps done. Holding for reset...")
        rospy.sleep(RESET_HOLD)
        self.publish_step(0.0, 0.0)

if __name__ == '__main__':
    try:
        loader = StepParamLoader()
        loader.run()
    except rospy.ROSInterruptException:
        pass
