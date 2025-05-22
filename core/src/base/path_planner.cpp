#include "wheeled_humanoid/base/path_planner.hpp"

namespace wheeled_humanoid::base {

PathPlanner::PathPlanner(int num_samples, double dt, double L,
                         double desired_velocity, const Dimensions dim)
    : num_samples_(num_samples), dt_(dt), L_(L),
      desired_velocity_(desired_velocity), dim_(dim) {
  dubins_radius_ = get_minimum_turning_radius(L, M_PI / 4);
}

std::vector<Obstacle>
PathPlanner::set_obstacles(const std::vector<Obstacle> obstacles) {
  obstacles_.clear();
  inflated_obstacles_.clear();
  std::vector<Obstacle> inflated_to_returns;

  namespace buffer = bg::strategy::buffer;
  buffer::distance_symmetric<double> distance_strategy((L_ / 2) * 1.2);
  buffer::side_straight side_strategy;
  buffer::join_miter join_strategy;
  buffer::end_flat end_strategy;
  buffer::point_circle point_strategy;

  for (const auto &obstacle : obstacles) {
    BoostPolygon original;
    bg::model::multi_polygon<BoostPolygon> inflated;
    Obstacle inflated_to_return;

    // Convert from custom Obstacle type to Boost polygon
    for (const auto &vertex : obstacle)
      bg::append(original, BoostPoint(vertex.x, vertex.y));
    bg::append(original, BoostPoint(obstacle.front().x, obstacle.front().y));

    bg::buffer(original, inflated, distance_strategy, side_strategy,
               join_strategy, end_strategy, point_strategy);

    if (inflated.size() > 0) {
      // Convert from Boost polygon to custom Obstacle type
      for (const auto &vertex : inflated[0].outer())
        inflated_to_return.push_back(
            Pose2D{bg::get<0>(vertex), bg::get<1>(vertex), 0.0});

      obstacles_.push_back(original);
      inflated_obstacles_.push_back(inflated[0]);
      inflated_to_returns.push_back(inflated_to_return);
    }
  }

  return inflated_to_returns;
}

DubinsPath PathPlanner::generate_path(const Pose2D &start,
                                      const Pose2D &goal) const {
  std::vector<Pose2D> points{start};
  std::vector<int> parent_idxs{-1};
  std::vector<double> distances{0};

  for (int sample = 0; sample < num_samples_; sample++) {
    sample_new_point(points, parent_idxs, distances, sample);
  }

  std::vector<double> goal_distances;
  for (int point_idx = 0; point_idx < points.size(); point_idx++) {
    auto distance_to_goal = get_euclidean_distance(points[point_idx], goal);
    goal_distances.push_back(distances[point_idx] + distance_to_goal);
  }

  int j = 0;
  std::vector<DubinsSegment> dubins_path;
  // TODO: Fix to get jth nearest point
  while (j < points.size()) {
    auto nearest_pt_idx =
        std::min_element(goal_distances.begin(), goal_distances.end()) -
        goal_distances.begin();
    auto nearest = points[nearest_pt_idx];
    auto dubins_seg_to_goal = get_dubins_segment(nearest, goal, dubins_radius_);

    if (check_collision(dubins_seg_to_goal)) {
      goal_distances[nearest_pt_idx] = std::numeric_limits<double>::max();
      j += 1;
    } else {
      dubins_path.push_back(dubins_seg_to_goal);
      auto parent_idx = parent_idxs[nearest_pt_idx];

      while (parent_idx != -1) {
        auto parent = points[parent_idx];
        auto path = get_dubins_segment(parent, nearest, dubins_radius_);

        dubins_path.push_back(path);
        parent_idx = parent_idxs[parent_idx];
        nearest = parent;
      }

      break;
    }
  }

  return DubinsPath(dubins_path.rbegin(), dubins_path.rend());
}

std::tuple<std::vector<Pose2D>, std::vector<int>, std::vector<double>>
PathPlanner::sample_new_point(std::vector<Pose2D> &points,
                              std::vector<int> &parent_idxs,
                              std::vector<double> &distances,
                              int sample) const {

  auto new_pt = create_halton_sample(sample);

  // Radius in which to search for nearest points
  // TODO: Decrease dynamically based on number of points
  auto proximity = get_euclidean_distance({dim_.x_min, dim_.y_min},
                                          {dim_.x_max, dim_.y_max});

  auto nearest_pt_idxs = get_nearest_neighbors(new_pt, points, proximity);

  std::vector<int> potential_parents;
  std::vector<DubinsSegment> potential_dubins_segs;
  std::vector<double> potential_distances;

  for (auto nearest_pt_idx : nearest_pt_idxs) {
    auto nearest_pt = points[nearest_pt_idx];

    auto dubins_seg = get_dubins_segment(nearest_pt, new_pt, dubins_radius_);

    if (!check_collision(dubins_seg)) {
      auto dist = dubins_seg.length;
      auto new_dist = dist + distances[nearest_pt_idx];

      potential_parents.push_back(nearest_pt_idx);
      potential_dubins_segs.push_back(dubins_seg);
      potential_distances.push_back(new_dist);
    }
  }

  if (potential_dubins_segs.empty()) {
    return {points, parent_idxs, distances};
  }

  auto min_idx =
      std::min_element(potential_distances.begin(), potential_distances.end()) -
      potential_distances.begin();
  auto dubins_seg = potential_dubins_segs[min_idx];
  auto new_dist = potential_distances[min_idx];
  auto new_parent_idx = potential_parents[min_idx];

  new_pt.theta = dubins_seg.line.end.theta;

  points.push_back(new_pt);
  parent_idxs.push_back(new_parent_idx);
  distances.push_back(new_dist);

  auto new_pt_idx = points.size() - 1;

  for (int j = 0; j < nearest_pt_idxs.size(); j++) {
    auto remaining_pt_idx = nearest_pt_idxs[j];
    if (remaining_pt_idx == new_parent_idx) {
      continue;
    }

    auto remaining_pt = points[remaining_pt_idx];
    auto remaining_dubins_seg =
        get_dubins_segment(new_pt, remaining_pt, dubins_radius_);
    auto nearest_to_new_dist = remaining_dubins_seg.length;

    if (new_dist + nearest_to_new_dist < distances[remaining_pt_idx] &&
        !check_collision(remaining_dubins_seg)) {
      points[remaining_pt_idx].theta = remaining_dubins_seg.line.end.theta;
      parent_idxs[remaining_pt_idx] = new_pt_idx;
      distances[remaining_pt_idx] = new_dist + nearest_to_new_dist;
    }
  }

  return {points, parent_idxs, distances};
}

// TODO: Fix bug where sometimes the robot velocity doubles on certain segments
Path PathPlanner::sample_path(const DubinsPath &dubins_path) const {
  if (dubins_path.empty()) {
    std::cerr << "Cannot interpolate path, original path is empty."
              << std::endl;
    return Path();
  }

  double full_path_length = 0;
  for (const auto &segment : dubins_path)
    full_path_length += segment.length;

  auto full_path_time = full_path_length / desired_velocity_;
  auto num_samples = std::ceil(full_path_time / dt_);
  auto min_distance_between_samples = desired_velocity_ * dt_;

  Path path{dubins_path.front().arc.start};

  for (const auto &segment : dubins_path) {
    auto num_arc_samples =
        std::ceil(segment.arc.length / full_path_length * num_samples);

    auto arc_samples = segment.arc.sample(num_arc_samples);
    for (const auto &sample : arc_samples) {
      if (get_euclidean_distance(sample, path.back()) >=
          min_distance_between_samples)
        path.push_back(sample);
    }

    auto num_line_samples =
        std::floor(segment.line.length / full_path_length * num_samples);

    auto line_samples = segment.line.sample(num_line_samples);
    for (const auto &sample : line_samples) {
      if (get_euclidean_distance(sample, path.back()) >=
          min_distance_between_samples)
        path.push_back(sample);
    }
  }

  return path;
}

VelocityProfile PathPlanner::get_velocity_profile(const Path &path) const {
  if (path.empty()) {
    std::cerr << "Cannot get velocity profile, path is empty." << std::endl;
    return VelocityProfile();
  }

  VelocityProfile velocity_profile;

  for (int i = 0; i < path.size() - 1; i++) {
    auto pt = path[i];
    auto next_pt = path[i + 1];

    auto dx = next_pt.x - pt.x;
    auto dy = next_pt.y - pt.y;

    double v = std::sqrt(dx * dx + dy * dy) / dt_;
    double omega = (next_pt.theta - pt.theta) / dt_;
    velocity_profile.push_back(BaseVelocity{v, omega});
  }
  velocity_profile.push_back(BaseVelocity{0, 0});

  return velocity_profile;
}

bool PathPlanner::check_collision(const DubinsSegment &segment) const {
  bg::model::linestring<BoostPoint> line;
  bg::append(line, BoostPoint(segment.line.start.x, segment.line.start.y));
  bg::append(line, BoostPoint(segment.line.end.x, segment.line.end.y));

  std::vector<BoostPoint> sampled_arc;
  auto arc_samples = segment.arc.sample(10);
  for (const auto &sample : arc_samples) {
    sampled_arc.push_back(BoostPoint(sample.x, sample.y));
  }

  for (const auto &obstacle : inflated_obstacles_) {
    if (bg::intersects(line, obstacle) || bg::within(line, obstacle))
      return true;

    for (const auto &sample : sampled_arc) {
      if (bg::within(sample, obstacle))
        return true;
    }
  }

  return false;
}

Pose2D PathPlanner::create_halton_sample(int index) const {
  auto radical_inverse = [](int index, int base) {
    double result = 0.0;
    double f = 1.0 / base;
    while (index > 0) {
      result += f * (index % base);
      index /= base;
      f /= base;
    }
    return result;
  };

  double x = radical_inverse(index, 2);
  double y = radical_inverse(index, 3);

  return Pose2D{
      x * (dim_.x_max - dim_.x_min) + dim_.x_min,
      y * (dim_.y_max - dim_.y_min) + dim_.y_min,
      0.0,
  };
}

std::vector<int> PathPlanner::get_nearest_neighbors(const Pose2D &new_pt,
                                                    const Path &points,
                                                    double radius) const {
  std::vector<int> nearest_pt_idxs;
  for (int i = 0; i < points.size(); i++) {
    auto dist = get_euclidean_distance(new_pt, points[i]);
    if (dist < radius) {
      nearest_pt_idxs.push_back(i);
    }
  }
  return nearest_pt_idxs;
}

} // namespace wheeled_humanoid::base