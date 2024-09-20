#!/usr/bin/env python                                                                                                                                                                                      

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg       import Float64

class Angular_Diff_Publisher:
    def __init__(self,robot_names):
        self.arm_pose_subscriber = rospy.Subscriber('body/ground_pose', Pose2D, self.store_arm_angular)
        self.bory_pose_subscriber = rospy.Subscriber('arm/ground_pose', Pose2D, self.store_body_angular)
        self.angular_diff_publisher = rospy.Publisher('/angular_diff', Float64, queue_size=1)
        self.arm_angular, self.body_angular = None, None

        self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_angular_diff)

    def store_arm_angular(msg):
        self.arm_angular = msg.theta

    def store_body_angular(msg):
        self.body_angular = msg.theta

    def pub_angular_diff(timer):
        diff = self.arm_angular - self.body_angular
        self.angular_diff_publisher.publish(diff)

if __name__ == '__main__':                                                                                                                                                                                 
     rospy.init_node('ang_diff_node')                                                                                                                                                                  
     velocity_command_publishers = Angular_Diff_Publisher(robot_names)                                                                                                                                   
     rospy.spin()  