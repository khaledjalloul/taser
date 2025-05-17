#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid::base {

enum Direction { LEFT, RIGHT };

struct Dimensions {
  double x_min, x_max;
  double y_min, y_max;
};

struct Circle {
  Pose2D center;
  double radius;
  Direction direction;
};

struct Arc {
  Pose2D start, end, center;
  Direction direction;
  double radius, arc_angle, length = 0;

  /**
   * Check for collision with given obstacles
   * @param obstacles Vector of obstacles in the environment
   * @return True if there is a collision with any obstacle, false otherwise
   */
  bool collides_with(const std::vector<Obstacle> &obstacles) const;
};

struct Line {
  Pose2D start;
  Pose2D end;
  double length = 0;

  /**
   * Check for collision with given obstacles
   * @param obstacles Vector of obstacles in the environment
   * @return True if there is a collision with any obstacle, false otherwise
   */
  bool collides_with(const std::vector<Obstacle> &obstacles) const;
};

struct DubinsSegment {

  Arc arc;
  Line line;

  double length();

  /**
   * Check for collision with given obstacles
   * @param obstacles Vector of obstacles in the environment
   * @return True if there is a collision with any obstacle, false otherwise
   */
  bool collides_with(const std::vector<Obstacle> &obstacles) const;
};

double get_car_turning_radius(double wheel_base, double max_steering_angle);

DubinsSegment get_dubins_segment(const Pose2D &start, const Pose2D &goal,
                                 double radius);

/**
 * Get the Euclidean distance between two points
 * @param a First point
 * @param b Second point
 * @return Euclidean distance between the two points
 */
double get_euclidean_distance(const Pose2D &a, const Pose2D &b);

std::vector<Line> get_tangent(const Circle &circle, const Pose2D &target);

std::tuple<Circle, Circle> get_turning_circles(const Pose2D &pose,
                                               double radius);

} // namespace wheeled_humanoid::base