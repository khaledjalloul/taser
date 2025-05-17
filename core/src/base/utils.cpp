#include "wheeled_humanoid/base/utils.hpp"

namespace wheeled_humanoid::base {

Path Arc::sample(int N) const {
  double r = hypot(start.x - center.x, start.y - center.y);
  double theta1 = atan2(start.y - center.y, start.x - center.x);
  double theta2 = atan2(end.y - center.y, end.x - center.x);

  // Normalize to [0, 2π)
  if (theta1 < 0)
    theta1 += 2 * M_PI;
  if (theta2 < 0)
    theta2 += 2 * M_PI;

  double delta = theta2 - theta1;
  if (direction == Direction::RIGHT && delta > 0)
    delta -= 2 * M_PI;
  else if (direction == Direction::LEFT && delta < 0)
    delta += 2 * M_PI;

  std::vector<Pose2D> samples;
  for (int i = 0; i <= N; i++) {
    double theta = theta1 + (delta * i / N);
    double x = center.x + r * cos(theta);
    double y = center.y + r * sin(theta);
    samples.push_back(Pose2D{x, y, theta});
  }
  return samples;
}

Path Line::sample(int N) const {
  std::vector<Pose2D> samples;
  for (int i = 0; i <= N; i++) {
    double t = static_cast<double>(i) / N;
    double x = start.x + t * (end.x - start.x);
    double y = start.y + t * (end.y - start.y);
    samples.push_back(Pose2D{x, y, start.theta});
  }
  return samples;
}

double get_car_turning_radius(double wheel_base, double max_steering_angle) {
  return wheel_base / std::tan(max_steering_angle);
}

DubinsSegment get_dubins_segment(const Pose2D &start, const Pose2D &goal,
                                 double radius) {
  auto start_circles_tup = get_turning_circles(start, radius);
  std::vector<Circle> start_circles = {std::get<0>(start_circles_tup),
                                       std::get<1>(start_circles_tup)};

  DubinsSegment seg;
  double length = std::numeric_limits<double>::max();

  for (const auto &circle : start_circles) {
    auto tangent = get_tangent(circle, goal);
    if (tangent.length == 0)
      continue;

    auto chord_length = get_euclidean_distance(start, tangent.start);
    if (chord_length > 2 * radius)
      continue;

    auto arc_angle = 2 * std::asin(chord_length / (2 * radius));

    auto orientation =
        (start.x - circle.center.x) * (tangent.start.y - circle.center.y) -
        (start.y - circle.center.y) * (tangent.start.x - circle.center.x);

    // Flip the arc angle if the orientation of the tangent is opposite to the
    // circle's
    if ((orientation > 0 && circle.direction == Direction::RIGHT) ||
        (orientation < 0 && circle.direction == Direction::LEFT)) {
      arc_angle = 2 * M_PI - arc_angle;
    }

    DubinsSegment temp_seg(Arc(start, tangent.start, circle.center,
                               circle.direction, radius, arc_angle),
                           Line(tangent.start, tangent.end));

    auto temp_length = temp_seg.length;
    if (temp_length < length) {
      seg = temp_seg;
      length = temp_length;
    }
  }

  return seg;
}

double get_euclidean_distance(const Pose2D &a, const Pose2D &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

Line get_tangent(const Circle &circle, const Pose2D &target) {
  auto C = circle.center;
  auto R = circle.radius;
  auto CP = get_euclidean_distance(C, target);
  auto direction = circle.direction;

  if (R > CP)
    return {};

  Eigen::Vector2d vec(target.x - C.x, target.y - C.y);
  auto th = std::acos(R / CP);

  Eigen::Matrix2d rot_mat;
  if (direction == Direction::RIGHT)
    rot_mat << std::cos(th), -std::sin(th), std::sin(th), std::cos(th);
  else
    rot_mat << std::cos(-th), -std::sin(-th), std::sin(-th), std::cos(-th);

  Eigen::Vector2d vec_perp = rot_mat * vec;
  vec_perp = Eigen::Vector2d(vec_perp(0) / CP * R, vec_perp(1) / CP * R);

  Pose2D tangent_pt{C.x + vec_perp(0), C.y + vec_perp(1)};
  auto theta = std::atan2(target.y - tangent_pt.y, target.x - tangent_pt.x);

  return Line(Pose2D{tangent_pt.x, tangent_pt.y, theta},
              Pose2D{target.x, target.y, theta});
}

std::tuple<Circle, Circle> get_turning_circles(const Pose2D &pose,
                                               double radius) {
  const double two_pi = 2.0 * M_PI;

  auto mod_2_pi = [&two_pi](double theta) {
    return theta - two_pi * std::floor(theta / two_pi);
  };

  auto theta = mod_2_pi(pose.theta);
  auto theta_left = mod_2_pi(theta + M_PI / 2);
  auto theta_right = mod_2_pi(theta - M_PI / 2);

  Circle left_circle{Pose2D{pose.x + radius * std::cos(theta_left),
                            pose.y + radius * std::sin(theta_left)},
                     radius, Direction::LEFT};
  Circle right_circle{Pose2D{pose.x + radius * std::cos(theta_right),
                             pose.y + radius * std::sin(theta_right)},
                      radius, Direction::RIGHT};
  return {left_circle, right_circle};
}

} // namespace wheeled_humanoid::base