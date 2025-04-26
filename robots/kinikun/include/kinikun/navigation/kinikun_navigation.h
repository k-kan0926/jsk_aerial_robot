// -*- mode c++ -*-
#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>

namespace aerial_robot_navigation
{
  class KinikunNavigator : public BaseNavigator
  {
  public:
    KinikunNavigator();
    ~KinikunNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;
    void reset() override;
    void startTakeoff() override;
  private:
    ros::Subscriber joy_sub_;
    ros::Publisher mpa_cmd_pub_;
    double mpa_cmd_x_, mpa_cmd_y_;
    double mpa_cmd_step_;
  };
};