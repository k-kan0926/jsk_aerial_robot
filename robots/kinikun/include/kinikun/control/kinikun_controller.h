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

#pragma once

#include <aerial_robot_control/control/under_actuated_controller.h>
#include <aerial_robot_msgs/PoseControlPid.h>
#include <geometry_msgs/WrenchStamped.h>
#include <boost/thread.hpp>
#include <mutex>

namespace aerial_robot_control
{
  enum WrenchCompIndex
  {
    FX = YAW + 1,
    FY,
    FZ,
    TX,
    TY,
    TZ,
  };

  class KinikunController : public UnderActuatedController
  {
  public:
    KinikunController();
    virtual ~KinikunController();

    void initialize(ros::NodeHandle nh,
                   ros::NodeHandle nhp,
                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                   double ctrl_loop_rate) override;

    void reset() override;

  protected:
    // External wrench estimation
    bool wrench_estimate_flag_;
    Eigen::VectorXd est_external_wrench_;
    Eigen::VectorXd init_sum_momentum_;
    Eigen::VectorXd integrate_term_;
    Eigen::MatrixXd momentum_observer_matrix_;
    double prev_est_wrench_timestamp_;
    boost::thread wrench_estimate_thread_;
    std::mutex wrench_mutex_;

    // Wrench compensation
    bool wrench_comp_mode_;
    double wrench_comp_p_gain_;
    double wrench_comp_i_gain_;
    double wrench_comp_d_gain_;
    double comp_term_update_freq_;
    double prev_comp_update_time_;
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;
    
    double I_comp_Fx_;
    double I_comp_Fy_;
    double I_comp_Fz_;
    double I_comp_Tx_;
    double I_comp_Ty_;
    double I_comp_Tz_;

    aerial_robot_msgs::PoseControlPid wrench_pid_msg_;

    // Publishers
    ros::Publisher estimate_external_wrench_pub_;
    ros::Publisher external_wrench_compensation_pub_;
    ros::Publisher wrench_comp_pid_pub_;

    // Core functions
    void controlCore() override;
    void sendCmd() override;
    virtual void externalWrenchEstimate();
    virtual void applyWrenchCompensation();
    virtual Eigen::VectorXd getTargetWrenchAccCog();
    
    void rosParamInit() override;
    void startWrenchEstimation();

    void cfgWrenchCompPidCallback(aerial_robot_control::PIDConfig &config, uint32_t level);

  };
};