#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState  # JointStateメッセージをインポート

class Angular_Diff_Publisher:
    def __init__(self):
        self.arm_pose_subscriber = rospy.Subscriber('arm/ground_pose', Pose2D, self.store_arm_angular)
        self.body_pose_subscriber = rospy.Subscriber('body/ground_pose', Pose2D, self.store_body_angular)
        # JointStateのパブリッシャーに変更
        self.angular_diff_publisher = rospy.Publisher('/quadrotor1/joint_states', JointState, queue_size=1)
        # self.angular_diff_publisher = rospy.Publisher('/quadrotor1/joints_ctrl', JointState, queue_size=1)
        self.arm_angular, self.body_angular = None, None

        self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_angular_diff)

    def store_arm_angular(self, msg):
        self.arm_angular = msg.theta

    def store_body_angular(self, msg):
        self.body_angular = msg.theta

    def pub_angular_diff(self, timer):
        if self.arm_angular is not None and self.body_angular is not None:
            # 角度差を計算
            angular_diff = self.arm_angular - self.body_angular

            # JointStateメッセージを作成して角度差をpositionに設定
            joint_state_msg = JointState()
            joint_state_msg.name = ['arm3_joint']
            joint_state_msg.position = [angular_diff]  # 角度差をpositionに設定
            # joint_state_msg.position = [0.0]  # 角度差をpositionに設定
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.velocity = [0.0]  # 速度を0に設定
            joint_state_msg.effort = [0.0]  # 力を0に設定
            

            # パブリッシュ
            self.angular_diff_publisher.publish(joint_state_msg)
            # rospy.loginfo(f"Published angular_diff as angle: {angular_diff} for arm3_joint")

if __name__ == '__main__':
    rospy.init_node('ang_diff_node')
    angular_diff_publisher = Angular_Diff_Publisher()
    rospy.spin()
