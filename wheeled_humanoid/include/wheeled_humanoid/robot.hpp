#pragma once

#include <map>
#include <string>

#include "wheeled_humanoid/arm_controller.hpp"
#include "wheeled_humanoid/arm_kinematics.hpp"
#include "wheeled_humanoid/base_controller.hpp"
#include "wheeled_humanoid/base_kinematics.hpp"
#include "wheeled_humanoid/rrt_path_planner.hpp"

namespace wheeled_humanoid {

class Robot {
public:
  std::tuple<ArmJointVelocities, double>
  move_arm_step(const std::string &arm_name, const Position3D &desired_position,
                const Transforms &tfs) const;

  void plan_path();

  std::tuple<double, double, double> follow_path_step();

  double dt = 0.1;

  std::map<std::string, ArmKinematics> arms = {
      {"left_arm", ArmKinematics("left_arm")},
      {"right_arm", ArmKinematics("right_arm")}};
  ArmController arm_controller{1.0};

  BaseKinematics base{0.5, 0.5, dt};
  BaseController base_controller{dt};

  RRTPathPlanner rrt_{100, dt, 6.0};
  Path planned_path;
  int planned_path_index = 0;
};

} // namespace wheeled_humanoid