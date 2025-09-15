#pragma once

#include "taser/types.hpp"

namespace taser::navigation {

/**
 * Get the Euclidean distance between two points
 * @param a First point
 * @param b Second point
 * @return Euclidean distance between the two points
 */
double get_euclidean_distance(const Pose2D &a, const Pose2D &b);

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
  double radius, angle, length;
  // Arc angle is always +ve if direction is LEFT
  // and -ve if direction is RIGHT

  Arc() = default;
  Arc(const Pose2D &start, const Pose2D &end, const Pose2D &center,
      Direction direction, double radius, double angle)
      : start(start), end(end), center(center), direction(direction),
        radius(radius), angle(angle) {
    length = radius * abs(angle);
  }

  /**
   * Sample N points along the arc
   * @param N Number of points to sample
   * @return Path of sampled points
   */
  Path sample(int N) const;
};

struct Line {
  Pose2D start;
  Pose2D end;
  double length;

  Line() = default;
  Line(const Pose2D &start, const Pose2D &end) : start(start), end(end) {
    length = get_euclidean_distance(start, end);
  }

  /**
   * Sample N points along the line
   * @param N Number of points to sample
   * @return Path of sampled points
   */
  Path sample(int N) const;
};

struct DubinsSegment {
  Arc arc;
  Line line;
  double length;

  DubinsSegment() = default;
  DubinsSegment(const Arc &arc, const Line &line) : arc(arc), line(line) {
    length = arc.length + line.length;
  }
};

using DubinsPath = std::vector<DubinsSegment>;

/**
 * Get the corners of a rectangular obstacle given its center and scale
 * @param x Center x coordinate
 * @param y Center y coordinate
 * @param scale_x Scale of the box in the x direction
 * @param scale_y Scale of the box in the y direction
 * @return Vector of the obstacle corners
 */
Obstacle get_box_corners(double x, double y, double scale_x, double scale_y);

/**
 * Get the Dubins segment composed of an arc and a line between two poses
 * @param start Start pose
 * @param goal Goal pose
 * @param radius Minimum turning radius
 * @return Dubins segment
 */
DubinsSegment get_dubins_segment(const Pose2D &start, const Pose2D &goal,
                                 double radius);

/**
 * Get the minimum turning radius for Dubins path calculation
 * @param wheel_base Distance between the wheels
 * @param max_steering_angle Maximum steering angle of the robot base
 * @return Minimum turning radius
 */
double get_minimum_turning_radius(double wheel_base, double max_steering_angle);

/**
 * Get the tangent line from an oriented circle to a point
 * @param circle Circle
 * @param target Target point
 * @return Tangent line
 */
Line get_tangent(const Circle &circle, const Pose2D &target);

/**
 * Get the two possible turning circles (left and right) at a given pose
 * @param pose Pose of the robot
 * @param radius Minimum turning radius
 * @return Tuple of the two turning circles
 */
std::tuple<Circle, Circle> get_turning_circles(const Pose2D &pose,
                                               double radius);

} // namespace taser::navigation