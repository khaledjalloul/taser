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

  std::tuple<std::vector<Pose2D>, std::vector<int>, std::vector<double>>
  sample_new_point(std::vector<Pose2D> &points, std::vector<int> &parent_idxs,
                   std::vector<double> &distances, const Dimensions &dim,
                   int sample) const;

  Path interpolate_path(const Path &path, int desired_n_points) const;

  bool check_line_collision(const Pose2D &a, const Pose2D &b) const;

  Dimensions get_dimensions(const Pose2D &start, const Pose2D &goal) const;

  Pose2D create_halton_sample(int index, const Dimensions &dim) const;

  double get_euclidean_distance(const Pose2D &a, const Pose2D &b) const;

  // TODO: Change to kd-tree
  std::vector<int> get_nearest_neighbors(const Pose2D &new_pt,
                                         const Path &points,
                                         double radius) const;

private:
  int num_samples_;
  std::vector<Obstacle> obstacles_;
};

} // namespace wheeled_humanoid
