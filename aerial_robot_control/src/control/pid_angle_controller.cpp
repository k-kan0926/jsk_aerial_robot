#include "aerial_robot_control/control/pid_angle_controller.h"

namespace aerial_robot_control
{
  AnglePIDController::AnglePIDController()
    : ControlBase(),
      angle_pid_("angle_pid"),
      target_angle_(0.0),
      current_angle_(0.0),
      p1_value_(0.2),
      p2_value_(0.2),
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
    current_angle_sub_ = nh_.subscribe("/arm3_joint", 1, &AnglePIDController::currentAngleCallback, this);

    // パブリッシャ
    p1p2_pub_ = nh_.advertise<geometry_msgs::Vector3>("p1p2_cmd", 10);

    // PIDゲインの初期化
    angle_pid_.setGains(1.0, 0.0, 0.1);  // P, I, Dゲイン
    angle_pid_.setLimits(VALUE_MAX, VALUE_MAX, VALUE_MAX, VALUE_MAX, ANGLE_MAX, ANGLE_MAX, ANGLE_MAX);

    initialized_ = true;
  }

  void AnglePIDController::targetAngleCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    target_angle_ = std::max(std::min(msg->data, ANGLE_MAX), ANGLE_MIN);
  }

  void AnglePIDController::currentAngleCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    if (!msg->name.empty() && msg->name[0] == "arm3_joint")
    {
      current_angle_ = std::max(std::min(msg->position[0], ANGLE_MAX), ANGLE_MIN);
    }
  }

  bool AnglePIDController::update()
  {
    if (!initialized_) return false;

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
    p1_value_ = std::clamp(control_effort > 0 ? VALUE_MIN + control_effort : VALUE_MIN, VALUE_MIN, VALUE_MAX);
    p2_value_ = std::clamp(control_effort < 0 ? VALUE_MIN - control_effort : VALUE_MIN, VALUE_MIN, VALUE_MAX);

    // Publish p1, p2 値
    geometry_msgs::Vector3 msg;
    msg.x = p1_value_;
    msg.y = p2_value_;
    p1p2_pub_.publish(msg);
  }
}
