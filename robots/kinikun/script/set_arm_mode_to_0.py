#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

DURATION = 12.0  # 目標値までの変化にかける秒数
FREQUENCY = 3    # 更新頻度（Hz）
DURATION2 = 5.0  # 目標値までの変化にかける秒数
FREQUENCY2 = 10   # 更新頻度（Hz）
ANGLE_THRESHOLD = 0.3  # positionが0度に近づいたと判断する閾値

class ParamLoader:
    def __init__(self):
        # Publisherの設定
        self.pub_v1v2 = rospy.Publisher('mpa_cmd', Vector3, queue_size=10)

        # Subscriberの設定
        rospy.Subscriber('/quadrotor1/joint_states', JointState, self.joint_state_callback)

        self.rate = rospy.Rate(FREQUENCY)  # 更新頻度で実行
        self.current_p1 = rospy.get_param('/p1_value', 0.0)
        self.current_p2 = rospy.get_param('/p2_value', 0.0)
        self.current_angle = None  # joint_statesのposition

    def load_yaml_file(self, file_path):
        """YAMLファイルを読み込み、パラメータを設定する関数"""
        try:
            with open(file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)
                if 'parameters' in yaml_data:
                    params = yaml_data['parameters']
                    p1_value = float(params['p1_value'])
                    p2_value = float(params['p2_value'])
                    rospy.loginfo(f"Loaded parameters from YAML: p1_value={p1_value}, p2_value={p2_value}")
                    return p1_value, p2_value
                else:
                    rospy.logwarn("YAML file does not contain 'parameters' field.")
                    return None, None
        except (FileNotFoundError, ValueError) as e:
            rospy.logwarn(f"YAML file not found or invalid format: {file_path}. Using current parameters.")
            return None, None
        except yaml.YAMLError as e:
            rospy.logerr(f"Error loading YAML file: {e}")
            return None, None

    def gradually_update(self, start, end, duration, frequency):
        """指定の開始と終了値の間を線形に補間してリストを返す"""
        steps = int(duration * frequency)
        return [(start + (end - start) * (i / steps)) for i in range(steps + 1)]

    def joint_state_callback(self, msg):
        """/joint_statesからpositionを取得するコールバック"""
        if 'arm3_joint' in msg.name:
            index = msg.name.index('arm3_joint')
            self.current_angle = msg.position[index]

    def update_mpa_cmd(self, target_p1, target_p2):
        """p1_valueとp2_valueを目標値までゆっくり更新"""
        p1_values = self.gradually_update(self.current_p1, target_p1, DURATION, FREQUENCY)
        p2_values = self.gradually_update(self.current_p2, target_p2, DURATION, FREQUENCY)

        for p1, p2 in zip(p1_values, p2_values):
            if self.current_angle is not None and abs(self.current_angle) < ANGLE_THRESHOLD:
                rospy.loginfo(f"Angle {self.current_angle} is within threshold. Switching to alternate YAML file.")
                return

            v1_value = p1 * 4096 / 0.9
            v2_value = p2 * 4096 / 0.9

            mpa_cmd_msg = Vector3(x=v1_value, y=v2_value, z=0.0)
            rospy.set_param('/p1_value', p1)
            rospy.set_param('/p2_value', p2)

            self.pub_v1v2.publish(mpa_cmd_msg)
            self.rate.sleep()  # 周期的に実行

        self.current_p1, self.current_p2 = target_p1, target_p2
    
    def update2_mpa_cmd(self, target_p1, target_p2):
        self.current_p1 = rospy.get_param('/p1_value', self.current_p1)
        self.current_p2 = rospy.get_param('/p2_value', self.current_p2)
        p1_values = self.gradually_update(self.current_p1, target_p1, DURATION, FREQUENCY)
        p2_values = self.gradually_update(self.current_p2, target_p2, DURATION, FREQUENCY)

        for p1, p2 in zip(p1_values, p2_values):
            v1_value = p1 * 4096 / 0.9
            v2_value = p2 * 4096 / 0.9

            mpa_cmd_msg = Vector3(x=v1_value, y=v2_value, z=0.0)
            rospy.set_param('/p1_value', p1)
            rospy.set_param('/p2_value', p2)

            self.pub_v1v2.publish(mpa_cmd_msg)
            self.rate.sleep()

        self.current_p1, self.current_p2 = target_p1, target_p2

    def run(self):
        # 初回のYAMLファイルのパス
        yaml_file_path = rospy.get_param('~yaml_file_path', None)
        alternate_yaml_file = rospy.get_param('~alternate_yaml_file', None)

        # 最初のYAMLファイルを処理
        if yaml_file_path:
            rospy.loginfo(f"Loading initial parameters from: {yaml_file_path}")
            target_p1, target_p2 = self.load_yaml_file(yaml_file_path)
            if target_p1 is not None and target_p2 is not None:
                self.update_mpa_cmd(target_p1, target_p2)

        # サブスクライブした角度が閾値以下になったら別のYAMLファイルを処理
        while not rospy.is_shutdown():
            if self.current_angle is not None and abs(self.current_angle) < ANGLE_THRESHOLD:
                rospy.loginfo("Angle is close to 0 degrees. Switching to alternate YAML file.")
                rospy.loginfo(f"Alternate YAML file path: {alternate_yaml_file}")

                if alternate_yaml_file:
                    target_p1, target_p2 = self.load_yaml_file(alternate_yaml_file)
                    if target_p1 is not None and target_p2 is not None:
                        self.update2_mpa_cmd(target_p1, target_p2)
                rospy.loginfo("All tasks completed. Exiting.")
                break

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('param_loader', anonymous=True)
    loader = ParamLoader()
    try:
        loader.run()
    except rospy.ROSInterruptException:
        pass
