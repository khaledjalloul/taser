#include "wheeled_humanoid/base/utils.hpp"

namespace wheeled_humanoid::base {

// TODO
bool Arc::collides_with(const std::vector<Obstacle> &obstacles) const {

  // bool DubinsPathPlanner::check_arc_collision(const DubinsArc& arc, double
  // resolution) const {
  //     double angle_diff = arc.end_angle - arc.start_angle;
  //     if (arc.is_left) {
  //         if (angle_diff < 0) angle_diff += 2 * M_PI;
  //     } else {
  //         if (angle_diff > 0) angle_diff -= 2 * M_PI;
  //     }

  //     int num_checks = std::ceil(std::abs(angle_diff * arc.radius) /
  //     resolution); if (num_checks < 2) num_checks = 2;

  //     Pose2D prev_point;
  //     for (int i = 0; i <= num_checks; ++i) {
  //         double t = static_cast<double>(i) / num_checks;
  //         double angle = arc.start_angle + t * angle_diff;

  //         Pose2D point;
  //         point.x = arc.center.x + arc.radius * std::cos(angle);
  //         point.y = arc.center.y + arc.radius * std::sin(angle);
  //         point.theta = angle + (arc.is_left ? M_PI/2 : -M_PI/2);

  //         if (i > 0 && collision_check_(prev_point, point)) {
  //             return true;  // Collision detected
  //         }
  //         prev_point = point;
  //     }
  //     return false;  // No collision
  // }

  return false;
}

bool Line::collides_with(const std::vector<Obstacle> &obstacles) const {
  const Pose2D &a = start;
  const Pose2D &b = end;

  auto orientation = [](const Pose2D &a, const Pose2D &b,
                        const Pose2D &c) -> int {
    double val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
    if (std::abs(val) < 1e-9)
      return 0;
    return (val > 0) ? 1 : 2;
  };

  auto on_segment = [](const Pose2D &p, const Pose2D &q,
                       const Pose2D &r) -> bool {
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
  };

  for (auto obstacle : obstacles) {
    for (int v_idx = 0; v_idx < obstacle.size(); v_idx++) {
      auto v2_idx = v_idx < obstacle.size() - 1 ? v_idx + 1 : 0;

      auto v = obstacle[v_idx];
      auto v2 = obstacle[v2_idx];

      int o1 = orientation(a, b, v);
      int o2 = orientation(a, b, v2);
      int o3 = orientation(v, v2, a);
      int o4 = orientation(v, v2, b);

      if (o1 != o2 && o3 != o4)
        return true;

      if (o1 == 0 && on_segment(a, v, b))
        return true;
      if (o2 == 0 && on_segment(a, v2, b))
        return true;
      if (o3 == 0 && on_segment(v, a, v2))
        return true;
      if (o4 == 0 && on_segment(v, b, v2))
        return true;
    }
  }

  return false;
}

bool DubinsSegment::collides_with(
    const std::vector<Obstacle> &obstacles) const {
  return arc.collides_with(obstacles) || line.collides_with(obstacles);
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
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

Line get_tangent(const Circle &circle, const Pose2D &target) {
  auto C1 = circle.center;
  auto R = circle.radius;
  auto C1P = get_euclidean_distance(C1, target);
  auto direction = circle.direction;

  if (R > C1P)
    return {};

  Eigen::Vector2d vec(target.x - C1.x, target.y - C1.y);
  auto th = std::acos(R / C1P);

  Eigen::Matrix2d rot_mat;
  if (direction == Direction::RIGHT)
    rot_mat << std::cos(th), -std::sin(th), std::sin(th), std::cos(th);
  else
    rot_mat << std::cos(-th), -std::sin(-th), std::sin(-th), std::cos(-th);

  Eigen::Vector2d vec_perp = rot_mat * vec;
  vec_perp = Eigen::Vector2d(vec_perp(0) / C1P * R, vec_perp(1) / C1P * R);

  Pose2D tangent_pt{C1.x + vec_perp(0), C1.y + vec_perp(1)};
  Eigen::Vector2d tangent_vec(target.x - tangent_pt.x, target.y - tangent_pt.y);
  auto theta = std::atan2(tangent_vec(1), tangent_vec(1));

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