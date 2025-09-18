#include "taser_cpp/navigation/geometry.hpp"

namespace taser_cpp::navigation {

double mod_2_pi(double theta) {
  return theta - (2 * M_PI) * std::floor(theta / (2 * M_PI));
};

double get_euclidean_distance(const Pose2D &a, const Pose2D &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

Path Arc::sample(int N) const {
  double r = get_euclidean_distance(start, center);
  double start_angle = atan2(start.y - center.y, start.x - center.x);

  double delta_theta = end.theta - start.theta;
  if (direction == Direction::LEFT && delta_theta < 0)
    delta_theta += 2 * M_PI;
  else if (direction == Direction::RIGHT && delta_theta > 0)
    delta_theta -= 2 * M_PI;

  std::vector<Pose2D> samples;
  for (int i = 0; i < N; i++) {
    double t = static_cast<double>(i) / (N - 1);
    double x = center.x + r * cos(start_angle + (t * angle));
    double y = center.y + r * sin(start_angle + (t * angle));
    double theta = start.theta + t * delta_theta;
    samples.push_back(Pose2D{x, y, theta});
  }
  return samples;
}

Path Line::sample(int N) const {
  std::vector<Pose2D> samples;
  for (int i = 0; i < N; i++) {
    double t = static_cast<double>(i) / (N - 1);
    double x = start.x + t * (end.x - start.x);
    double y = start.y + t * (end.y - start.y);
    samples.push_back(Pose2D{x, y, start.theta});
  }
  return samples;
}

Obstacle get_box_corners(double x, double y, double scale_x, double scale_y) {
  Pose2D front_left{x + scale_x / 2, y - scale_y / 2};
  Pose2D front_right{x + scale_x / 2, y + scale_y / 2};
  Pose2D back_left{x - scale_x / 2, y - scale_y / 2};
  Pose2D back_right{x - scale_x / 2, y + scale_y / 2};
  // Apparently this order of vertices is important for the Boost buffer to work
  return {back_left, back_right, front_right, front_left};
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

    auto start_theta =
        atan2(start.y - circle.center.y, start.x - circle.center.x);
    auto end_theta = atan2(tangent.start.y - circle.center.y,
                           tangent.start.x - circle.center.x);

    double arc_angle = end_theta - start_theta;

    if (circle.direction == Direction::RIGHT && arc_angle > 0)
      arc_angle -= 2 * M_PI;
    else if (circle.direction == Direction::LEFT && arc_angle < 0)
      arc_angle += 2 * M_PI;

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

double get_minimum_turning_radius(double wheel_base,
                                  double max_steering_angle) {
  return wheel_base / std::tan(max_steering_angle);
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

} // namespace taser_cpp::navigation