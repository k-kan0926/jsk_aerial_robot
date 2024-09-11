#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import JointState

def load_yaml_file(file_path):
    """YAMLファイルを読み込み、パラメータを設定する関数"""
    try:
        with open(file_path, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            if 'parameters' in yaml_data:
                params = yaml_data['parameters']
                rospy.set_param('/p1_value', params['p1_value'])
                rospy.set_param('/p2_value', params['p2_value'])
                # YAMLから取得したangle値をfloatに変換して返す
                angle = float(params['angle'])
                rospy.set_param('/angle', angle)
                rospy.loginfo(f"Parameters set: p1_value={params['p1_value']}, p2_value={params['p2_value']}, angle={angle}")
                return angle  # angle値を返す
            else:
                rospy.logwarn("YAML file does not contain 'parameters' field.")
                return None
    except (FileNotFoundError, ValueError) as e:
        rospy.logwarn(f"YAML file not found or invalid format: {file_path}. Using default parameters.")
        return None
    except yaml.YAMLError as e:
        rospy.logerr(f"Error loading YAML file: {e}")
        return None

def main():
    # ROSノードの初期化
    rospy.init_node('param_loader', anonymous=True)

    # デフォルトのパラメータを設定
    default_p1 = 0.3
    default_p2 = 0.3
    default_angle = -0.1  # デフォルトの角度

    rospy.set_param('/p1_value', default_p1)  # デフォルトのp1_value
    rospy.set_param('/p2_value', default_p2)  # デフォルトのp2_value
    rospy.set_param('/angle', default_angle)  # デフォルトの角度

    rospy.loginfo(f"Default parameters set: p1_value={default_p1}, p2_value={default_p2}, angle={default_angle}")

    # ユーザーが指定したモード (YAMLファイル) のパスを引数で取得
    yaml_file_path = rospy.get_param('~yaml_file_path', None)

    if yaml_file_path:
        rospy.loginfo(f"Loading parameters from: {yaml_file_path}")
        # YAMLファイルを読み込んでパラメータを設定
        angle = load_yaml_file(yaml_file_path)
    else:
        rospy.loginfo("No YAML file provided. Using default angle.")
        angle = default_angle

    # 値がNoneでないことを確認してfloat型に変換
    if angle is None:
        angle = default_angle
    else:
        angle = float(angle)

    # JointStateメッセージを準備してトピックに一度だけPublish
    pub = rospy.Publisher('/quadrotor/joints_ctrl', JointState, queue_size=10)

    # JointStateメッセージを作成
    joint_state_msg = JointState()
    joint_state_msg.name = ['arm1_joint']  # 対象のジョイント名
    joint_state_msg.position = [angle]  # 指定された角度 (必ずfloat型に変換)
    joint_state_msg.velocity = [0.0]  # 速度を0に設定
    joint_state_msg.effort = [0.0]  # 力を0に設定

    # 1回だけPublishして終了
    rospy.sleep(1)  # 少し待ってからパブリッシュ
    joint_state_msg.header.stamp = rospy.Time.now()
    pub.publish(joint_state_msg)
    rospy.loginfo(f"Published angle: {angle} for arm1_joint")
    
    rospy.sleep(1)  # 終了前に少し待機

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
