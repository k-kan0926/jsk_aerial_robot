// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <kinikun/control/kinikun_controller.h>
#include <kinikun/sensor/imu.h>


using namespace std;

namespace aerial_robot_control
{
  KinikunController::KinikunController():
    UnderActuatedController(),
    wrench_estimate_flag_(false),
    wrench_comp_mode_(false),
    prev_est_wrench_timestamp_(0),
    prev_comp_update_time_(-1),
    comp_term_update_freq_(10.0),
    I_comp_Fx_(0), I_comp_Fy_(0), I_comp_Fz_(0),
    I_comp_Tx_(0), I_comp_Ty_(0), I_comp_Tz_(0)
  {
    wrench_pid_msg_.x.total.resize(1);
    wrench_pid_msg_.x.p_term.resize(1);
    wrench_pid_msg_.x.i_term.resize(1);
    wrench_pid_msg_.x.d_term.resize(1);
    wrench_pid_msg_.y.total.resize(1);
    wrench_pid_msg_.y.p_term.resize(1);
    wrench_pid_msg_.y.i_term.resize(1);
    wrench_pid_msg_.y.d_term.resize(1);
    wrench_pid_msg_.z.total.resize(1);
    wrench_pid_msg_.z.p_term.resize(1);
    wrench_pid_msg_.z.i_term.resize(1);
    wrench_pid_msg_.z.d_term.resize(1);
    wrench_pid_msg_.roll.total.resize(1);
    wrench_pid_msg_.roll.p_term.resize(1);
    wrench_pid_msg_.roll.i_term.resize(1);
    wrench_pid_msg_.roll.d_term.resize(1);
    wrench_pid_msg_.pitch.total.resize(1);
    wrench_pid_msg_.pitch.p_term.resize(1);
    wrench_pid_msg_.pitch.i_term.resize(1);
    wrench_pid_msg_.pitch.d_term.resize(1);
    wrench_pid_msg_.yaw.total.resize(1);
    wrench_pid_msg_.yaw.p_term.resize(1);
    wrench_pid_msg_.yaw.i_term.resize(1);
    wrench_pid_msg_.yaw.d_term.resize(1);
  }

  KinikunController::~KinikunController()
  {
    if(wrench_estimate_flag_)
    {
      wrench_estimate_thread_.interrupt();
      wrench_estimate_thread_.join();
    }
  }

  void KinikunController::initialize(ros::NodeHandle nh,
                                                     ros::NodeHandle nhp,
                                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                     double ctrl_loop_rate)
  {
    UnderActuatedController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    ros::NodeHandle control_nh(nh_, "controller");
    
    // Load parameters
    rosParamInit();

    // Publishers
    estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
    external_wrench_compensation_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("external_wrench_compensation", 1);
    wrench_comp_pid_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("debug/wrench_comp/pid", 1);

    // Initialize PID controllers for wrench compensation
    if(wrench_comp_mode_)
    {
      pid_controllers_.push_back(PID("f_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
      pid_controllers_.push_back(PID("f_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
      pid_controllers_.push_back(PID("f_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
      pid_controllers_.push_back(PID("t_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
      pid_controllers_.push_back(PID("t_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
      pid_controllers_.push_back(PID("t_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
      
      // Dynamic reconfigure for wrench compensation gains
      ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
      std::vector<int> indices = {FX, FY, FZ, TX, TY, TZ};
      pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_nh));
      pid_reconf_servers_.back()->setCallback(boost::bind(&KinikunController::cfgPidCallback, this, _1, _2, indices));
      
      ROS_INFO("Wrench compensation mode enabled with PD control");
    }

    // Start external wrench estimation
    if(wrench_estimate_flag_)
    {
      startWrenchEstimation();
      ROS_INFO("External wrench estimation enabled");
    }
  }

  void KinikunController::reset()
  {
    UnderActuatedController::reset();

    if(wrench_comp_mode_)
    {
      pid_controllers_.at(FX).reset();
      pid_controllers_.at(FY).reset();
      pid_controllers_.at(FZ).reset();
      pid_controllers_.at(TX).reset();
      pid_controllers_.at(TY).reset();
      pid_controllers_.at(TZ).reset();
      
      pid_controllers_.at(X).setICompTerm(0.0);
      pid_controllers_.at(Y).setICompTerm(0.0);
      pid_controllers_.at(Z).setICompTerm(0.0);
      pid_controllers_.at(ROLL).setICompTerm(0.0);
      pid_controllers_.at(PITCH).setICompTerm(0.0);
      pid_controllers_.at(YAW).setICompTerm(0.0);
    }

    I_comp_Fx_ = 0;
    I_comp_Fy_ = 0;
    I_comp_Fz_ = 0;
    I_comp_Tx_ = 0;
    I_comp_Ty_ = 0;
    I_comp_Tz_ = 0;
    prev_comp_update_time_ = -1;
  }

  void KinikunController::controlCore()
  {
    // Apply wrench compensation before normal control
    if(wrench_comp_mode_ && 
       (navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE ||
        navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE))
    {
      applyWrenchCompensation();
    }
    else
    {
      // Reset compensation when not hovering/taking off
      if(wrench_comp_mode_)
      {
        pid_controllers_.at(FX).reset();
        pid_controllers_.at(FY).reset();
        pid_controllers_.at(FZ).reset();
        pid_controllers_.at(TX).reset();
        pid_controllers_.at(TY).reset();
        pid_controllers_.at(TZ).reset();
        
        pid_controllers_.at(X).setICompTerm(0.0);
        pid_controllers_.at(Y).setICompTerm(0.0);
        pid_controllers_.at(Z).setICompTerm(0.0);
        pid_controllers_.at(ROLL).setICompTerm(0.0);
        pid_controllers_.at(PITCH).setICompTerm(0.0);
        pid_controllers_.at(YAW).setICompTerm(0.0);
      }
    }

    // Call parent control core
    UnderActuatedController::controlCore();
  }

  void KinikunController::sendCmd()
  {
    UnderActuatedController::sendCmd();

    // Publish wrench compensation PID info
    if(wrench_comp_mode_)
    {
      wrench_comp_pid_pub_.publish(wrench_pid_msg_);
    }
  }

  void KinikunController::applyWrenchCompensation()
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    
    double mass_inv = 1.0 / robot_model_->getMass();
    Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
    
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
    
    // Get estimated external wrench in COG frame
    Eigen::VectorXd wrench_cog = est_external_wrench_;
    wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);
    
    // Clamp wrench values to prevent excessive compensation
    for(int i = 0; i < 6; i++)
    {
      if(wrench_cog(i) > external_wrench_upper_limit_(i))
        wrench_cog(i) = external_wrench_upper_limit_(i);
      if(wrench_cog(i) < external_wrench_lower_limit_(i))
        wrench_cog(i) = external_wrench_lower_limit_(i);
    }
    
    // Convert wrench to acceleration for I-term reconfiguration method
    Eigen::VectorXd I_reconfig_acc_cog_term = Eigen::VectorXd::Zero(6);
    I_reconfig_acc_cog_term.head(3) = mass_inv * wrench_cog.head(3);
    I_reconfig_acc_cog_term.tail(3) = inertia_inv * wrench_cog.tail(3);
    
    // Get I gains from main PID controllers
    double IGain_Fx = pid_controllers_.at(X).getIGain();
    double IGain_Fy = pid_controllers_.at(Y).getIGain();
    double IGain_Fz = pid_controllers_.at(Z).getIGain();
    double IGain_Tx = pid_controllers_.at(ROLL).getIGain();
    double IGain_Ty = pid_controllers_.at(PITCH).getIGain();
    double IGain_Tz = pid_controllers_.at(YAW).getIGain();
    
    // Calculate time difference
    double du;
    if(prev_comp_update_time_ < 0)
    {
      prev_comp_update_time_ = ros::Time::now().toSec();
      return;
    }
    else
    {
      du = ros::Time::now().toSec() - prev_comp_update_time_;
      prev_comp_update_time_ = ros::Time::now().toSec();
    }
    
    // Update wrench compensation PID controllers using I-term reconfiguration method
    // The error input is the required change in acceleration divided by I-gain
    if(fabs(IGain_Fx) > 1e-6) 
      pid_controllers_.at(FX).updateWoVel(I_reconfig_acc_cog_term(0) / IGain_Fx, du);
    if(fabs(IGain_Fy) > 1e-6) 
      pid_controllers_.at(FY).updateWoVel(I_reconfig_acc_cog_term(1) / IGain_Fy, du);
    if(fabs(IGain_Fz) > 1e-6) 
      pid_controllers_.at(FZ).updateWoVel(I_reconfig_acc_cog_term(2) / IGain_Fz, du);
    if(fabs(IGain_Tx) > 1e-6) 
      pid_controllers_.at(TX).updateWoVel(I_reconfig_acc_cog_term(3) / IGain_Tx, du);
    if(fabs(IGain_Ty) > 1e-6) 
      pid_controllers_.at(TY).updateWoVel(I_reconfig_acc_cog_term(4) / IGain_Ty, du);
    if(fabs(IGain_Tz) > 1e-6) 
      pid_controllers_.at(TZ).updateWoVel(I_reconfig_acc_cog_term(5) / IGain_Tz, du);
    
    // Get compensation terms
    I_comp_Fx_ = pid_controllers_.at(FX).result();
    I_comp_Fy_ = pid_controllers_.at(FY).result();
    I_comp_Fz_ = pid_controllers_.at(FZ).result();
    I_comp_Tx_ = pid_controllers_.at(TX).result();
    I_comp_Ty_ = pid_controllers_.at(TY).result();
    I_comp_Tz_ = pid_controllers_.at(TZ).result();
    
    // Apply compensation to main PID controllers' I-term
    pid_controllers_.at(X).setICompTerm(I_comp_Fx_);
    pid_controllers_.at(Y).setICompTerm(I_comp_Fy_);
    pid_controllers_.at(Z).setICompTerm(I_comp_Fz_);
    pid_controllers_.at(ROLL).setICompTerm(I_comp_Tx_);
    pid_controllers_.at(PITCH).setICompTerm(I_comp_Ty_);
    pid_controllers_.at(YAW).setICompTerm(I_comp_Tz_);
    
    // Publish compensation acceleration info
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = I_reconfig_acc_cog_term(0);
    wrench_msg.wrench.force.y = I_reconfig_acc_cog_term(1);
    wrench_msg.wrench.force.z = I_reconfig_acc_cog_term(2);
    wrench_msg.wrench.torque.x = I_reconfig_acc_cog_term(3);
    wrench_msg.wrench.torque.y = I_reconfig_acc_cog_term(4);
    wrench_msg.wrench.torque.z = I_reconfig_acc_cog_term(5);
    external_wrench_compensation_pub_.publish(wrench_msg);
    
    // Update wrench PID message for debugging
    wrench_pid_msg_.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    
    wrench_pid_msg_.x.total.at(0) = pid_controllers_.at(FX).result();
    wrench_pid_msg_.x.p_term.at(0) = pid_controllers_.at(FX).getPTerm();
    wrench_pid_msg_.x.i_term.at(0) = pid_controllers_.at(FX).getITerm();
    wrench_pid_msg_.x.d_term.at(0) = pid_controllers_.at(FX).getDTerm();
    
    wrench_pid_msg_.y.total.at(0) = pid_controllers_.at(FY).result();
    wrench_pid_msg_.y.p_term.at(0) = pid_controllers_.at(FY).getPTerm();
    wrench_pid_msg_.y.i_term.at(0) = pid_controllers_.at(FY).getITerm();
    wrench_pid_msg_.y.d_term.at(0) = pid_controllers_.at(FY).getDTerm();
    
    wrench_pid_msg_.z.total.at(0) = pid_controllers_.at(FZ).result();
    wrench_pid_msg_.z.p_term.at(0) = pid_controllers_.at(FZ).getPTerm();
    wrench_pid_msg_.z.i_term.at(0) = pid_controllers_.at(FZ).getITerm();
    wrench_pid_msg_.z.d_term.at(0) = pid_controllers_.at(FZ).getDTerm();
    
    wrench_pid_msg_.roll.total.at(0) = pid_controllers_.at(TX).result();
    wrench_pid_msg_.roll.p_term.at(0) = pid_controllers_.at(TX).getPTerm();
    wrench_pid_msg_.roll.i_term.at(0) = pid_controllers_.at(TX).getITerm();
    wrench_pid_msg_.roll.d_term.at(0) = pid_controllers_.at(TX).getDTerm();
    
    wrench_pid_msg_.pitch.total.at(0) = pid_controllers_.at(TY).result();
    wrench_pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(TY).getPTerm();
    wrench_pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(TY).getITerm();
    wrench_pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(TY).getDTerm();
    
    wrench_pid_msg_.yaw.total.at(0) = pid_controllers_.at(TZ).result();
    wrench_pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(TZ).getPTerm();
    wrench_pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(TZ).getITerm();
    wrench_pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(TZ).getDTerm();
  }

  Eigen::VectorXd KinikunController::getTargetWrenchAccCog()
  {
    Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
    
    Eigen::Matrix3d cog_rot_eigen;
    tf::Matrix3x3 cog_rot_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);
    tf::matrixTFToEigen(cog_rot_tf, cog_rot_eigen);
    
    // Get target acceleration in world frame from PID controllers
    tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                            pid_controllers_.at(Y).result(),
                            pid_controllers_.at(Z).result());
    
    // Convert to COG frame
    Eigen::Vector3d target_acc_cog_eigen;
    tf::vectorTFToEigen(cog_rot_tf.inverse() * target_acc_w, target_acc_cog_eigen);
    target_wrench_acc_cog.head(3) = target_acc_cog_eigen;
    
    // Angular acceleration targets (roll, pitch, yaw in COG frame)
    target_wrench_acc_cog(3) = pid_controllers_.at(ROLL).result();
    target_wrench_acc_cog(4) = pid_controllers_.at(PITCH).result();
    target_wrench_acc_cog(5) = pid_controllers_.at(YAW).result();
    
    return target_wrench_acc_cog;
  }

  void KinikunController::externalWrenchEstimate()
  {
    const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();

    if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }
    
    if(target_wrench_acc_cog.size() == 0)
    {
      ROS_WARN_THROTTLE(5.0, "Target wrench value for wrench estimation is not set.");
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

    // Use filtered velocity and angular velocity from IMU
    Eigen::Vector3d vel_w, omega_cog;
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::KinikunImu>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
    
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    double mass = robot_model_->getMass();

    // Calculate current momentum
    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;

    // Target wrench in COG frame
    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
    target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);

    // Transformation matrix (world to COG frame for forces)
    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6, 6);
    J_t.topLeftCorner(3, 3) = cog_rot;

    // Nonlinear terms (gravity and gyroscopic effects)
    Eigen::VectorXd N = mass * robot_model_->getGravity();
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

    if(prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum;
      return;
    }

    double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

    // Momentum observer integration
    integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;

    // Estimate external wrench using momentum observer
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);
    }

    // Convert estimated wrench to COG frame for publishing
    Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
    est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);

    // Publish estimated external wrench
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = est_external_wrench_cog(0);
    wrench_msg.wrench.force.y = est_external_wrench_cog(1);
    wrench_msg.wrench.force.z = est_external_wrench_cog(2);
    wrench_msg.wrench.torque.x = est_external_wrench_cog(3);
    wrench_msg.wrench.torque.y = est_external_wrench_cog(4);
    wrench_msg.wrench.torque.z = est_external_wrench_cog(5);
    estimate_external_wrench_pub_.publish(wrench_msg);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();
  }

  void KinikunController::startWrenchEstimation()
  {
    est_external_wrench_ = Eigen::VectorXd::Zero(6);
    init_sum_momentum_ = Eigen::VectorXd::Zero(6);
    integrate_term_ = Eigen::VectorXd::Zero(6);
    momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6, 6);
    prev_est_wrench_timestamp_ = 0;

    double force_weight, torque_weight;
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<double>(control_nh, "momentum_observer_force_weight", force_weight, 10.0);
    getParam<double>(control_nh, "momentum_observer_torque_weight", torque_weight, 10.0);
    momentum_observer_matrix_.topRows(3) *= force_weight;
    momentum_observer_matrix_.bottomRows(3) *= torque_weight;

    wrench_estimate_thread_ = boost::thread([this]()
    {
      double update_rate;
      ros::NodeHandle control_nh(nh_, "controller");
      control_nh.param("wrench_estimate_update_rate", update_rate, 100.0);

      ros::Rate loop_rate(update_rate);
      while(ros::ok())
      {
        externalWrenchEstimate();
        loop_rate.sleep();
      }
    });
  }

  void KinikunController::rosParamInit()
  {
    UnderActuatedController::rosParamInit();

    ros::NodeHandle control_nh(nh_, "controller");
    
    // External wrench estimation parameters
    getParam<bool>(control_nh, "wrench_estimate_flag", wrench_estimate_flag_, false);
    
    // Wrench compensation parameters
    getParam<bool>(control_nh, "wrench_comp_mode", wrench_comp_mode_, false);
    
    if(wrench_comp_mode_)
    {
      ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
      getParam<double>(wrench_nh, "p_gain", wrench_comp_p_gain_, 0.1);
      getParam<double>(wrench_nh, "i_gain", wrench_comp_i_gain_, 0.005);
      getParam<double>(wrench_nh, "d_gain", wrench_comp_d_gain_, 0.07);
      
      double external_force_upper_limit, external_force_lower_limit;
      double external_torque_upper_limit, external_torque_lower_limit;
      getParam<double>(control_nh, "external_force_upper_limit", external_force_upper_limit, 5.0);
      getParam<double>(control_nh, "external_force_lower_limit", external_force_lower_limit, -5.0);
      getParam<double>(control_nh, "external_torque_upper_limit", external_torque_upper_limit, 0.5);
      getParam<double>(control_nh, "external_torque_lower_limit", external_torque_lower_limit, -0.5);
      
      external_wrench_upper_limit_ = Eigen::VectorXd::Zero(6);
      external_wrench_lower_limit_ = Eigen::VectorXd::Zero(6);
      external_wrench_upper_limit_.head(3) = Eigen::Vector3d::Constant(external_force_upper_limit);
      external_wrench_upper_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_upper_limit);
      external_wrench_lower_limit_.head(3) = Eigen::Vector3d::Constant(external_force_lower_limit);
      external_wrench_lower_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_lower_limit);
      
      getParam<double>(control_nh, "comp_term_update_freq", comp_term_update_freq_, 10.0);
      
      ROS_INFO_STREAM("External wrench upper limit: " << external_wrench_upper_limit_.transpose());
      ROS_INFO_STREAM("External wrench lower limit: " << external_wrench_lower_limit_.transpose());
      ROS_INFO_STREAM("Wrench compensation gains - P: " << wrench_comp_p_gain_ 
                      << " I: " << wrench_comp_i_gain_ << " D: " << wrench_comp_d_gain_);
    }
  }

} // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::KinikunController, aerial_robot_control::ControlBase);