#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

struct Dimensions {
  double x_min, x_max;
  double y_min, y_max;
};

struct Obstacle {
  double x, y;
  double radius;
};

class RRTPathPlanner {
public:
  RRTPathPlanner(double dt, double T_total);

  Path generate_path(const Pose2D &start, const Pose2D &goal) const;

  Path interpolate_path(const Path &path) const;

private:
  Dimensions get_dimensions_(const Pose2D &start, const Pose2D &goal) const;

  Pose2D create_halton_sample_(int index, const Dimensions &dim) const;

  double get_euclidean_distance_(const Pose2D &a, const Pose2D &b) const;

  // TODO: Change to kd-tree
  std::vector<int> get_nearest_neighbors_(const Pose2D &new_pt,
                                          const Path &points,
                                          double radius) const;

  // TODO
  bool check_line_collision_(const Pose2D &a, const Pose2D &b) const;

  double dt_, T_total_;
  int num_samples_;
  std::vector<Obstacle> obstacles_;
};

} // namespace wheeled_humanoid
