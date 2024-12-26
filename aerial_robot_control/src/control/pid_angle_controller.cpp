#include "aerial_robot_control/control/pid_angle_controller.h"

namespace aerial_robot_control
{
  AnglePIDController::AnglePIDController()
    : ControlBase(),
      angle_pid_("angle_pid"),
      target_angle_(0.0),
      current_angle_(0.0),
      p1_value_(VALUE_MIN),
      p2_value_(VALUE_MIN),
      initialized_(false)
  {}

  void AnglePIDController::initialize(ros::NodeHandle nh,
                                       ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_du)
  {
    ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

    // サブスクライバ
    target_angle_sub_ = nh_.subscribe("/target_angle", 1, &AnglePIDController::targetAngleCallback, this);
    current_angle_sub_ = nh_.subscribe("/quadrotor/joint_states", 1, &AnglePIDController::currentAngleCallback, this);

    // パブリッシャ
    p1p2_pub_ = nh_.advertise<geometry_msgs::Vector3>("mpa_cmd", 10);

    // PIDゲインの初期化
    angle_pid_.setGains(1.0, 0.0, 0.1); // P, I, Dゲイン
    angle_pid_.setLimits(VALUE_MIN, VALUE_MAX, VALUE_MIN, VALUE_MAX, ANGLE_MIN, ANGLE_MAX, ANGLE_MAX);

    // 初期値設定
    current_angle_ = 0.0; // 初期化時は安全な値
    target_angle_ = 0.0;
    p1_value_ = VALUE_MIN;
    p2_value_ = VALUE_MIN;

    initialized_ = false; // サブスクライバから値を受け取るまで未初期化とする
  }

  void AnglePIDController::targetAngleCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    target_angle_ = std::clamp(static_cast<double>(msg->data), ANGLE_MIN, ANGLE_MAX);
  }

  void AnglePIDController::currentAngleCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), "arm3_joint");
    if (it != msg->name.end())
    {
      size_t index = std::distance(msg->name.begin(), it);
      current_angle_ = std::clamp(static_cast<double>(msg->position[index]), ANGLE_MIN, ANGLE_MAX);
      initialized_ = true; // 値を初めて受け取ったときに初期化完了とする
    }
  }

  bool AnglePIDController::update()
  {
    if (!initialized_)
    {
      ROS_WARN_THROTTLE(1.0, "Waiting for current_angle_ to be updated...");
      return false;
    }

    // PID制御を実行
    controlPID();
    return true;
  }

  void AnglePIDController::controlPID()
  {
    double error = target_angle_ - current_angle_;
    angle_pid_.update(error, ctrl_loop_du_, 0);

    double control_effort = angle_pid_.result();

    // p1_value と p2_value の計算
    p1_value_ = std::clamp(VALUE_MIN + (control_effort > 0 ? control_effort : 0), VALUE_MIN, VALUE_MAX);
    p2_value_ = std::clamp(VALUE_MIN - (control_effort < 0 ? -control_effort : 0), VALUE_MIN, VALUE_MAX);

    // Publish p1, p2 値
    geometry_msgs::Vector3 msg;
    msg.x = p1_value_;
    msg.y = p2_value_;
    p1p2_pub_.publish(msg);

    // デバッグ用ログ
    ROS_INFO_THROTTLE(1.0, "Target Angle: %.2f, Current Angle: %.2f, P1: %.2f, P2: %.2f",
                      target_angle_, current_angle_, p1_value_, p2_value_);
  }
}
