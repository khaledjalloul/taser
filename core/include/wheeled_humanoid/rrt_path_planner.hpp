#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid {

struct Dimensions {
  double x_min, x_max;
  double y_min, y_max;
};

/**
 * RRT* path planner
 */
class RRTPathPlanner {
public:
  /**
   * @param num_samples Number of samples to generate when searching for a path
   * @param dt Time step
   * @param L Distance between the robot wheels
   */
  RRTPathPlanner(int num_samples, double dt, double L);

  /**
   * Set the obstacles in the environment and inflate them based on the distance
   * between the robot wheels
   * @param obstacles List of obstacles
   * @return Inflated obstacles (Also stored in a member variable)
   */
  std::vector<Obstacle> set_obstacles(const std::vector<Obstacle> obstacles);

  /**
   * Generate a path (vector of poses) from start to goal pose
   * @param start Start pose
   * @param goal Goal pose
   * @return Path from start to goal
   */
  Path generate_path(const Pose2D &start, const Pose2D &goal) const;

  /**
   * Sample a new point in the environment, used internally in `generate_path`
   * @param[in,out] points List of sampled points in the environment so far
   * @param[in,out] parent_idxs List of parent indices for each point
   * @param[in,out] distances List of distances from start to each point
   * @param[in] dim Dimensions of the environment
   * @param[in] sample Index of the sample to generate
   * @return Tuple of new points, parent indices, and distances (Only useful in
   * Python bindings)
   */
  std::tuple<std::vector<Pose2D>, std::vector<int>, std::vector<double>>
  sample_new_point(std::vector<Pose2D> &points, std::vector<int> &parent_idxs,
                   std::vector<double> &distances, const Dimensions &dim,
                   int sample) const;

  /**
   * Interpolate the path to get a smoother trajectory
   * @param path Path to interpolate
   * @param desired_n_points Number of points to interpolate to
   * @return Interpolated path
   */
  Path interpolate_path(const Path &path, int desired_n_points) const;

  /**
   * Get the velocity profile (vector of base velocities) for the path
   * @param path Path to get the velocity profile for
   * @return Velocity profile for the path
   */
  VelocityProfile get_velocity_profile(const Path &path) const;

  /**
   * Check if a line segment between two points collides with any obstacles in
   * the environment
   * @param a Start point of the line segment
   * @param b End point of the line segment
   * @return True if there is a collision, false otherwise
   */
  bool check_line_collision(const Pose2D &a, const Pose2D &b) const;

  /**
   * Get the dimensions of the environment based on the start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return Dimensions of the environment
   */
  Dimensions get_dimensions(const Pose2D &start, const Pose2D &goal) const;

  /**
   * Create a Halton sample in the environment
   * @param index Index of the sample
   * @param dim Dimensions of the environment
   * @return Halton sample
   */
  Pose2D create_halton_sample(int index, const Dimensions &dim) const;

  /**
   * Get the Euclidean distance between two points
   * @param a First point
   * @param b Second point
   * @return Euclidean distance between the two points
   */
  double get_euclidean_distance(const Pose2D &a, const Pose2D &b) const;

  /**
   * Get the nearest neighbors of a point in the environment. TODO: Change to
   * kd-tree
   * @param new_pt Point to find neighbors for
   * @param points List of points in the environment
   * @param radius Radius to search for neighbors
   * @return List of indices of the nearest neighbors
   */
  std::vector<int> get_nearest_neighbors(const Pose2D &new_pt,
                                         const Path &points,
                                         double radius) const;

private:
  int num_samples_;
  double dt_, L_;
  std::vector<Obstacle> obstacles_, inflated_obstacles_;
};

} // namespace wheeled_humanoid
