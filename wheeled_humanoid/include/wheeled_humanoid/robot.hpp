#pragma once

#include <map>
#include <string>

#include "wheeled_humanoid/arm_controller.hpp"
#include "wheeled_humanoid/arm_kinematics.hpp"
#include "wheeled_humanoid/base_controller.hpp"
#include "wheeled_humanoid/base_kinematics.hpp"

namespace wheeled_humanoid {

class Robot {
public:
  std::tuple<ArmJointVelocities, double>
  get_arm_dq_step(const std::string &arm_name,
                  const Position3D &desired_position,
                  const Transforms &tfs) const;

  std::map<std::string, ArmKinematics> arms = {
      {"left_arm", ArmKinematics("left_arm")},
      {"right_arm", ArmKinematics("right_arm")}};

  double dt = 0.1;
  BaseKinematics base{0.5, 0.5, dt};

private:
  ArmController arm_controller_{1.0};
  BaseController base_controller_{dt};
};

} // namespace wheeled_humanoid