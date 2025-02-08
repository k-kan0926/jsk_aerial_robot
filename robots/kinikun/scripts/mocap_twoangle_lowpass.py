#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState

class Angular_Diff_Publisher:
    def __init__(self):
        self.arm1_pose_subscriber = rospy.Subscriber('arm1/ground_pose', Pose2D, self.store_arm1_angular)
        self.arm2_pose_subscriber = rospy.Subscriber('arm2/ground_pose', Pose2D, self.store_arm2_angular)
        self.body_pose_subscriber = rospy.Subscriber('body/ground_pose', Pose2D, self.store_body_angular)
        self.angular_diff_publisher = rospy.Publisher('/quadrotor1/joint_states', JointState, queue_size=1)

        # 角度差のフィルタ後の値を格納する変数
        self.filtered_angular_diff1 = None
        self.filtered_angular_diff2 = None
        self.alpha = 0.1  # フィルタ係数

        self.arm1_angular, self.arm2_angular, self.body_angular = None, None, None
        self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_angular_diff)

    def store_arm1_angular(self, msg):
        self.arm1_angular = msg.theta

    def store_arm2_angular(self, msg):
        self.arm2_angular = msg.theta

    def store_body_angular(self, msg):
        self.body_angular = msg.theta

    def pub_angular_diff(self, timer):
        if self.arm1_angular is not None and self.arm2_angular is not None and self.body_angular is not None:
            # 生の角度差を計算
            angular_diff1 = self.arm1_angular - self.body_angular
            angular_diff2 = self.arm2_angular - self.body_angular

            # フィルタ適用
            if self.filtered_angular_diff1 is None:  # 初回は生の値をそのまま使用
                self.filtered_angular_diff1 = angular_diff1
                self.filtered_angular_diff2 = angular_diff2
            else:
                self.filtered_angular_diff1 = self.alpha * angular_diff1 + (1 - self.alpha) * self.filtered_angular_diff1
                self.filtered_angular_diff2 = self.alpha * angular_diff2 + (1 - self.alpha) * self.filtered_angular_diff2

            # JointStateメッセージの作成
            joint_state_msg1 = JointState()
            joint_state_msg1.name = ['arm3_joint']
            joint_state_msg1.position = [self.filtered_angular_diff1]
            joint_state_msg1.header.stamp = rospy.Time.now()

            joint_state_msg2 = JointState()
            joint_state_msg2.name = ['arm2_joint']
            joint_state_msg2.position = [self.filtered_angular_diff2]
            joint_state_msg2.header.stamp = rospy.Time.now()

            # パブリッシュ
            self.angular_diff_publisher.publish(joint_state_msg1)
            self.angular_diff_publisher.publish(joint_state_msg2)
            rospy.loginfo(f"Filtered angular_diff1: {self.filtered_angular_diff1}")
            rospy.loginfo(f"Filtered angular_diff2: {self.filtered_angular_diff2}")


if __name__ == '__main__':
    rospy.init_node('ang_diff_node')
    angular_diff_publisher = Angular_Diff_Publisher()
    rospy.spin()
