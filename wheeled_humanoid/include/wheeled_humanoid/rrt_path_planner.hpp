#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

class RRTPathPlanner {
public:
  RRTPathPlanner(double dt, double T_total);

  Path interpolate_path(const Path &path);

private:
  double dt_, T_total_;
};

} // namespace wheeled_humanoid