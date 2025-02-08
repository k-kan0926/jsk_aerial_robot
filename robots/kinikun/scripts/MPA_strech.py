#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

DURATION = 5.0  # 目標値までの変化にかける秒数
FREQUENCY = 10  # 更新頻度（Hz）

# 初期のYAMLファイルパスを絶対パスで指定
current_yaml_file = "/home/keiichiro/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/config/arm_mode3.yaml"

def load_yaml_file(file_path):
    """YAMLファイルを読み込み、パラメータを設定する関数"""
    try:
        with open(file_path, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            if 'parameters' in yaml_data:
                params = yaml_data['parameters']
                p1_value = float(params['p1_value'])
                p2_value = float(params['p2_value'])
                angle = float(params['angle'])
                rospy.loginfo(f"Loaded parameters from YAML: p1_value={p1_value}, p2_value={p2_value}, angle={angle}")
                return p1_value, p2_value, angle
            else:
                rospy.logwarn("YAML file does not contain 'parameters' field.")
                return None, None, None
    except (FileNotFoundError, ValueError) as e:
        rospy.logwarn(f"YAML file not found or invalid format: {file_path}. Using current parameters.")
        return None, None, None
    except yaml.YAMLError as e:
        rospy.logerr(f"Error loading YAML file: {e}")
        return None, None, None

def gradually_update(start, end, duration, frequency):
    """指定の開始と終了値の間を線形に補間してリストを返す"""
    steps = int(duration * frequency)
    return [(start + (end - start) * (i / steps)) for i in range(steps)]

def main():
    global current_yaml_file
    rospy.init_node('param_loader', anonymous=True)

    # Publisherの設定
    pub = rospy.Publisher('/quadrotor/joint_states', JointState, queue_size=10)
    pub_v1v2 = rospy.Publisher('mpa_cmd', Vector3, queue_size=10)

    rate = rospy.Rate(FREQUENCY)  # 更新頻度で実行

    while not rospy.is_shutdown():
        rospy.loginfo(f"Loading parameters from: {current_yaml_file}")
        target_p1, target_p2, target_angle = load_yaml_file(current_yaml_file)

        if target_p1 is None or target_p2 is None or target_angle is None:
            rospy.logwarn("Invalid target values, skipping this iteration.")
            continue

        # 現在の値を取得
        current_p1 = rospy.get_param('/p1_value', 0.0)
        current_p2 = rospy.get_param('/p2_value', 0.0)
        current_angle = rospy.get_param('/angle', 0.0)

        # 目標値に達しているか確認
        if abs(current_p1 - target_p1) < 0.01 and abs(current_p2 - target_p2) < 0.01 and abs(current_angle - target_angle) < 0.01:
            # YAMLファイルを切り替える
            current_yaml_file = "/home/keiichiro/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot//robots/kinikun/config/arm_mode4.yaml" if current_yaml_file == "/home/keiichiro/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot//robots/kinikun/config/arm_mode3.yaml" else "/home/keiichiro/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/config/arm_mode3.yaml"
            rospy.loginfo(f"Target values reached. Switching to {current_yaml_file}")
            continue  # 次のループで新しいYAMLファイルを読み込み

        # 補間値リストを生成
        p1_values = gradually_update(current_p1, target_p1, DURATION, FREQUENCY)
        p2_values = gradually_update(current_p2, target_p2, DURATION, FREQUENCY)
        angle_values = gradually_update(current_angle, target_angle, DURATION, FREQUENCY)

        for p1, p2, angle in zip(p1_values, p2_values, angle_values):
            joint_state_msg = JointState()
            joint_state_msg.name = ['arm3_joint']
            joint_state_msg.position = [angle]

            v1_value = p1 * 4096 / 0.9
            v2_value = p2 * 4096 / 0.9

            mpa_cmd_msg = Vector3()
            mpa_cmd_msg.x = v1_value
            mpa_cmd_msg.y = v2_value

            # パラメータの更新とメッセージのPublish
            rospy.set_param('/p1_value', p1)
            rospy.set_param('/p2_value', p2)
            rospy.set_param('/angle', angle)

            joint_state_msg.header.stamp = rospy.Time.now()
            pub.publish(joint_state_msg)
            rospy.loginfo(f"Published angle: {angle} for arm3_joint")

            pub_v1v2.publish(mpa_cmd_msg)
            rospy.loginfo(f"Published V1={v1_value}, V2={v2_value}")

            rate.sleep()  # 周期的に実行

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
