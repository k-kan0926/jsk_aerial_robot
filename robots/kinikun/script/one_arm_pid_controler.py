#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math
from dynamic_reconfigure.server import Server
from kinikun.cfg import PIDConfig

class PIDController:
    def __init__(self):
        # パラメータ
        self.kp = rospy.get_param('~kp', 1.0)
        self.ki = rospy.get_param('~ki', 0.1)
        self.kd = rospy.get_param('~kd', 0.1)
        self.rate = rospy.get_param('~rate', 50)  # 制御周期 (Hz)

        # 内部状態
        self.target_angle = 0.0
        self.current_angle = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        self.server = Server(PIDConfig, self.dynamic_reconfigure_callback)

        # サブスクライバー
        rospy.Subscriber('/quadrotor1/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/target_angle', Float64, self.target_angle_callback)

        # パブリッシャー
        self.mpa_cmd_publisher = rospy.Publisher('mpa_cmd', Vector3, queue_size=10)

        # 制御ループのタイマー
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.control_loop)

    def dynamic_reconfigure_callback(self, config, level):
        """
        Dynamic Reconfigureのコールバック
        """
        self.kp = config.p_gain
        self.ki = config.i_gain
        self.kd = config.d_gain
        self.target_angle = config.target_angle  # target_angleもrqt_reconfigureから変更可能
        rospy.loginfo(f"Reconfigure Request: P={self.kp}, I={self.ki}, D={self.kd}, Target Angle={self.target_angle}")
        return config

    def joint_state_callback(self, msg):
        try:
            # `arm3_joint`の角度を取得
            index = msg.name.index('arm2_joint')
            self.current_angle = msg.position[index]
            self.current_angle = math.degrees(msg.position[index])  # ラジアンを度に変換
        except ValueError:
            rospy.logwarn("arm3_joint not found in JointState message")

    def target_angle_callback(self, msg):
        self.target_angle = msg.data

    def control_loop(self, event):
        # PID制御の計算
        error = self.target_angle - self.current_angle
        self.integral += error / self.rate
        derivative = (error - self.prev_error) * self.rate

        # 制御信号の計算
        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # 制御信号をmpa_cmdに変換 (制限付き)
        p1_value = max(0.2, min(0.6, control_signal))
        p2_value = max(0.2, min(0.6, -control_signal))  # p2は逆方向の値と仮定

        v1_value = p1_value * 4096 / 0.9
        v2_value = p2_value * 4096 / 0.9

        # メッセージ作成とパブリッシュ
        mpa_cmd_msg = Vector3()
        mpa_cmd_msg.x = v1_value
        mpa_cmd_msg.y = v2_value
        mpa_cmd_msg.z = 0.0  # 必要に応じて使用
        self.mpa_cmd_publisher.publish(mpa_cmd_msg)

        rospy.loginfo(f"Target: {self.target_angle}, Current: {self.current_angle}, V1: {v1_value}, V2: {v2_value}")

if __name__ == '__main__':
    rospy.init_node('pid_controller')
    try:
        PIDController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
