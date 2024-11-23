// angle_pid.h
#pragma once

#include "aerial_robot_control/control/utils/pid.h"
#include <algorithm>  // std::clamp

namespace aerial_robot_control
{
  class AnglePIDController : public PID {
  public:
    AnglePIDController(const std::string& name, double p_gain, double i_gain, double d_gain,
                       double limit_sum, double limit_p, double limit_i, double limit_d)
        : PID(name, p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d) {}

    void updateAngleControl(const double target_angle, const double current_angle, double& p1_value, double& p2_value, double du) {
      double error_angle = std::clamp(target_angle - current_angle, -45.0, 45.0);  // エラーにリミット設定
      update(error_angle, du, 0, 0);  // PID計算
      double control_output = result();

      // 電圧の更新とクリッピング
      p1_value = std::clamp(p1_value + control_output, 900.0, 2800.0);
      p2_value = std::clamp(p2_value - control_output, 900.0, 2800.0);
    }
  };
};
