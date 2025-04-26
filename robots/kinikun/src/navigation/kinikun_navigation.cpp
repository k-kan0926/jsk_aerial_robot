#include <kinikun/navigation/kinikun_navigation.h>

using namespace aerial_robot_navigation;
using namespace aerial_robot_model;

KinikunNavigator::KinikunNavigator():
  BaseNavigator(),
  v1_mpa_(0.2),
  v2_mpa_(0.2),
  step_(0.02),
  v_min_(0.0),
  v_max_(0.7),
{

}

void KinikunNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  double loop_du)
{
  /*intialize the flight control*/
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, &KinikunNavigator::joyCallback, this);
  mpa_cmd_pub_ = nh.advertise<geometry_msgs::Vector3>("mpa_cmd", 1);
  ROS_INFO("[KinikunNavigator] initialized.");

}

void KinikunNavigator::joyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd = (*joy_msg);

  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_2])
  
}