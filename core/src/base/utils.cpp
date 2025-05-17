#include "wheeled_humanoid/base/utils.hpp"

namespace wheeled_humanoid::base {

// TODO
bool Arc::collides_with(const std::vector<Obstacle> &obstacles) const {
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

double DubinsSegment::length() { return arc.length + line.length; }

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

  for (const auto &start_circle : start_circles) {
    auto tangent = get_tangent(start_circle, goal);

    auto arc_length1 = std::sqrt(std::pow(start.x - tangent[0].start.x, 2) +
                                 std::pow(start.y - tangent[0].start.y, 2));
    if (arc_length1 > 2 * radius)
      continue;

    auto arc_angle1 =
        std::acos(2 * std::pow(radius, 2) -
                  std::pow(arc_length1, 2) / (2 * std::pow(radius, 2)));

    auto c1 = (start.x - start_circle.center.x) *
                  (tangent[0].start.y + start_circle.center.y) -
              (start.y - start_circle.center.y) *
                  (tangent[0].start.x - start_circle.center.x);

    if ((c1 > 0 && start_circle.direction == Direction::RIGHT) ||
        (c1 < 0 && start_circle.direction == Direction::LEFT))
      arc_angle1 = 2 * M_PI - arc_angle1;

    auto line_len =
        std::sqrt(std::pow(tangent[0].start.x - tangent[0].end.x, 2) +
                  std::pow(tangent[0].start.y - tangent[0].end.y, 2));

    DubinsSegment temp_seg{Arc{start, tangent[0].start, start_circle.center,
                               start_circle.direction, radius, arc_angle1,
                               radius * arc_angle1},
                           Line{tangent[0].start, tangent[0].end, line_len}};

    auto temp_length = temp_seg.length();
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

std::vector<Line> get_tangent(const Circle &circle, const Pose2D &target) {
  auto C1 = circle.center;
  auto R = circle.radius;
  auto C1P =
      std::sqrt(std::pow(C1.x - target.x, 2) + std::pow(C1.y - target.y, 2));
  auto direction = circle.direction;

  if (R > C1P)
    return {};

  std::vector<Line> tangents;

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

  tangents.push_back(Line{Pose2D{tangent_pt.x, tangent_pt.y, theta},
                          Pose2D{target.x, target.y, theta}});

  return tangents;
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