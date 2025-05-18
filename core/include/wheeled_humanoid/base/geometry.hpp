#pragma once

#include "wheeled_humanoid/types.hpp"

namespace wheeled_humanoid::base {

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

Obstacle get_box_corners(double x, double y, double scale_x, double scale_y);

double get_car_turning_radius(double wheel_base, double max_steering_angle);

DubinsSegment get_dubins_segment(const Pose2D &start, const Pose2D &goal,
                                 double radius);

Line get_tangent(const Circle &circle, const Pose2D &target);

std::tuple<Circle, Circle> get_turning_circles(const Pose2D &pose,
                                               double radius);

} // namespace wheeled_humanoid::base