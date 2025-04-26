#include "wheeled_humanoid/rrt_path_planner.hpp"

namespace wheeled_humanoid {

RRTPathPlanner::RRTPathPlanner(double dt, double T_total)
    : dt_(dt), T_total_(T_total) {}

Path RRTPathPlanner::interpolate_path(const Path &path) {
  int n_points = path.size();
  int n_steps = static_cast<int>(T_total_ / dt_);

  Path ref_path;

  std::vector<double> t_path(n_points);
  for (int i = 0; i < n_points; ++i)
    t_path[i] = static_cast<double>(i) / (n_points - 1);

  for (int i = 0; i < n_steps; ++i) {
    double t = static_cast<double>(i) * dt_ / T_total_;

    // Clamp to [0, 1]
    t = std::min(std::max(t, 0.0), 1.0);

    // Find segment index j such that t_path[j] <= t < t_path[j+1]
    int j = 0;
    while (j < n_points - 2 && t > t_path[j + 1])
      ++j;

    double t0 = t_path[j];
    double t1 = t_path[j + 1];
    double alpha = (t - t0) / (t1 - t0);

    Pose2D new_pt{(1 - alpha) * path[j].x + alpha * path[j + 1].x,
                  (1 - alpha) * path[j].y + alpha * path[j + 1].y,
                  (1 - alpha) * path[j].theta + alpha * path[j + 1].theta};

    ref_path.push_back(new_pt);
  }

  return ref_path;
}

} // namespace wheeled_humanoid