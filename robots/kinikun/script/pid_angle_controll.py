import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState


class TargetAnglePIDController:
    def __init__(self):
        self.target_angle = 0.0
        self.current_angle = 0.0

        # ROS Subscribers
        self.target_angle_sub = rospy.Subscriber('/target_angle', Float32, self.target_angle_callback)
        self.current_angle_sub = rospy.Subscriber('/quadrotor/joint_states', JointState, self.current_angle_callback)

        # ROS Publisher
        self.p1p2_pub = rospy.Publisher('/p1p2_cmd', Vector3, queue_size=10)

        # PID Gains
        self.p_gain = rospy.get_param('~p_gain', 1.0)
        self.i_gain = rospy.get_param('~i_gain', 0.0)
        self.d_gain = rospy.get_param('~d_gain', 0.1)

    def target_angle_callback(self, msg):
        self.target_angle = msg.data

    def current_angle_callback(self, msg):
        if 'arm3_joint' in msg.name:
            index = msg.name.index('arm3_joint')
            self.current_angle = msg.position[index]

    def control(self):
        # Calculate Error
        error = self.target_angle - self.current_angle

        # Simple PID
        p1_value = self.p_gain * error if error > 0 else 0.2
        p2_value = -self.p_gain * error if error < 0 else 0.2

        # Publish p1 and p2
        msg = Vector3()
        msg.x = p1_value
        msg.y = p2_value
        self.p1p2_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('angle_pid_controller')
    controller = TargetAnglePIDController()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        controller.control()
        rate.sleep()
