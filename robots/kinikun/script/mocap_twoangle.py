#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState  # JointStateメッセージをインポート

class Angular_Diff_Publisher:
    def __init__(self):
        self.arm1_pose_subscriber = rospy.Subscriber('arm1/ground_pose', Pose2D, self.store_arm1_angular)
        self.arm2_pose_subscriber = rospy.Subscriber('arm2/ground_pose', Pose2D, self.store_arm2_angular)        
        self.body_pose_subscriber = rospy.Subscriber('body/ground_pose', Pose2D, self.store_body_angular)
        # JointStateのパブリッシャーに変更
        self.angular_diff_publisher = rospy.Publisher('/quadrotor1/joint_states', JointState, queue_size=1)
        # self.angular_diff_publisher = rospy.Publisher('/quadrotor1/joints_ctrl', JointState, queue_size=1)
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
            # 角度差を計算
            angular_diff1 = self.arm1_angular - self.body_angular
            angular_diff2 = self.arm2_angular - self.body_angular

            # JointStateメッセージを作成して角度差をpositionに設定
            joint_state_msg1 = JointState()
            joint_state_msg1.name = ['arm3_joint']
            joint_state_msg1.position = [angular_diff1] 
            joint_state_msg1.header.stamp = rospy.Time.now()
            joint_state_msg1.velocity = [0.0]  
            joint_state_msg1.effort = [0.0]  
            
            joint_state_msg2 = JointState()
            joint_state_msg2.name = ['arm4_joint']
            joint_state_msg2.position = [angular_diff2] 
            joint_state_msg2.header.stamp = rospy.Time.now()
            joint_state_msg2.velocity = [0.0]  
            joint_state_msg2.effort = [0.0]  
            
            # パブリッシュ
            self.angular_diff_publisher.publish(joint_state_msg1)
            self.angular_diff_publisher.publish(joint_state_msg2)
            # rospy.loginfo(f"Published angular_diff as angle: {angular_diff} for arm3_joint")

if __name__ == '__main__':
    rospy.init_node('ang_diff_node')
    angular_diff_publisher = Angular_Diff_Publisher()
    rospy.spin()
