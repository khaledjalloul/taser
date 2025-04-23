#pragma once

#include <map>
#include <string>

#include "wheeled_humanoid/arm_controller.hpp"
#include "wheeled_humanoid/arm_kinematics.hpp"
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
  BaseKinematics base{0.5, 0.5, 0.1};

private:
  ArmController arm_controller_{1.0};
};

} // namespace wheeled_humanoid