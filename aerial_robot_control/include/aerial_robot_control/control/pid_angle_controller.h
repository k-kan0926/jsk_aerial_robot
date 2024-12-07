#pragma once

#include <aerial_robot_control/control/base/base.h>
#include <aerial_robot_control/control/utils/pid.h>
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_control/PIDConfig.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

namespace aerial_robot_control
{
  class AnglePIDController : public ControlBase
  {
  public:
    AnglePIDController();
    ~AnglePIDController() = default;

    void initialize(ros::NodeHandle nh,
                    ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_du) override;

    bool update() override;

  private:
    void targetAngleCallback(const std_msgs::Float32::ConstPtr& msg);
    void currentAngleCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void controlPID();

    PID angle_pid_;
    double target_angle_;
    double current_angle_;
    double p1_value_, p2_value_;

    ros::Subscriber target_angle_sub_;
    ros::Subscriber current_angle_sub_;
    ros::Publisher p1p2_pub_;

    bool initialized_;

    static constexpr double ANGLE_MIN = -0.7854;
    static constexpr double ANGLE_MAX = 0.7854;
    static constexpr double VALUE_MIN = 0.2;
    static constexpr double VALUE_MAX = 0.7;
  };
};
