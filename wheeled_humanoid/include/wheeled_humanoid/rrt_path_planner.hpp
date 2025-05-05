#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

struct Dimensions {
  double x_min, x_max;
  double y_min, y_max;
};

// An obstacle is represented as a polygon, defined by its vertices
using Obstacle = std::vector<Pose2D>;

class RRTPathPlanner {
public:
  RRTPathPlanner(int num_samples);

  void set_obstacles(const std::vector<Obstacle> obstacles);

  Path generate_path(const Pose2D &start, const Pose2D &goal) const;

  Path interpolate_path(const Path &path, int desired_n_points) const;

  bool check_line_collision(const Pose2D &a, const Pose2D &b) const;

private:
  Dimensions get_dimensions_(const Pose2D &start, const Pose2D &goal) const;

  Pose2D create_halton_sample_(int index, const Dimensions &dim) const;

  double get_euclidean_distance_(const Pose2D &a, const Pose2D &b) const;

  // TODO: Change to kd-tree
  std::vector<int> get_nearest_neighbors_(const Pose2D &new_pt,
                                          const Path &points,
                                          double radius) const;

  int num_samples_;
  std::vector<Obstacle> obstacles_;
};

} // namespace wheeled_humanoid
