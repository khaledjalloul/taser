#pragma once

#include <boost/geometry.hpp>

#include "wheeled_humanoid/base/geometry.hpp"
#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid::base {

namespace bg = boost::geometry;

/**
 * RRT* path planner
 */
class PathPlanner {
  using BoostPolygon = bg::model::polygon<bg::model::d2::point_xy<double>>;
  using BoostPoint = bg::model::d2::point_xy<double>;

public:
  /**
   * Constructor
   * @param num_samples Number of samples to generate when searching for a path
   * @param dt Time step
   * @param L Distance between the robot wheels
   * @param desired_velocity Desired velocity of the robot
   */
  PathPlanner(int num_samples, double dt, double L, double desired_velocity);

  /**
   * Set the obstacles in the environment and inflate them based on the distance
   * between the robot wheels
   * @param obstacles List of obstacles
   * @return Inflated obstacles (using custom Obstacle type instead of Boost)
   */
  std::vector<Obstacle> set_obstacles(const std::vector<Obstacle> obstacles);

  /**
   * Generate a path (vector of poses) from start to goal pose
   * @param start Start pose
   * @param goal Goal pose
   * @return Path from start to goal
   */
  DubinsPath generate_path(const Pose2D &start, const Pose2D &goal) const;

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
   * Sample a dubins path to get a vector of poses
   * @param dubins_path Dubins path
   * @return Sampled path
   */
  Path sample_path(const DubinsPath &dubins_path) const;

  /**
   * Get the velocity profile (vector of base velocities) for the path
   * @param path Path to get the velocity profile for
   * @return Velocity profile for the path
   */
  VelocityProfile get_velocity_profile(const Path &path) const;

  /**
   * Check if a dubins segment collides with any obstacles in the environment
   * @param segment Dubins segment
   * @return True if there is a collision, false otherwise
   */
  bool check_collision(const DubinsSegment &segment) const;

  /**
   * Create a Halton sample in the environment
   * @param index Index of the sample
   * @param dim Dimensions of the environment
   * @return Halton sample
   */
  Pose2D create_halton_sample(int index, const Dimensions &dim) const;

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
  double dt_, L_, dubins_radius_, desired_velocity_;
  std::vector<BoostPolygon> obstacles_, inflated_obstacles_;
};

} // namespace wheeled_humanoid::base
