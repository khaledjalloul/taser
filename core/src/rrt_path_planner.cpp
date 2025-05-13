#include "wheeled_humanoid/rrt_path_planner.hpp"

#include <clipper2/clipper.h>

#include <iostream>

namespace wheeled_humanoid {

RRTPathPlanner::RRTPathPlanner(int num_samples, double dt, double L)
    : num_samples_(num_samples), dt_(dt), L_(L) {}

std::vector<Obstacle>
RRTPathPlanner::set_obstacles(const std::vector<Obstacle> obstacles) {
  obstacles_ = obstacles;
  inflated_obstacles_.clear();

  for (const auto &obstacle : obstacles) {
    Clipper2Lib::PathD original, inflated;

    for (const auto &vertex : obstacle) {
      original.push_back(Clipper2Lib::PointD({vertex.x, vertex.y}));
    }

    inflated = Clipper2Lib::InflatePaths(
                   Clipper2Lib::PathsD{original}, (L_ / 2) * 1.2,
                   Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon)
                   .front();

    Obstacle inflated_obstacle;
    for (const auto &vertex : inflated) {
      inflated_obstacle.push_back(Pose2D{vertex.x, vertex.y});
    }

    inflated_obstacles_.push_back(inflated_obstacle);
  }

  return inflated_obstacles_;
}

Path RRTPathPlanner::generate_path(const Pose2D &start,
                                   const Pose2D &goal) const {
  std::vector<Pose2D> points{start};
  std::vector<int> parent_idxs{-1};
  std::vector<double> distances{0};

  auto dim = get_dimensions(start, goal);

  for (int sample = 0; sample < num_samples_; sample++) {
    sample_new_point(points, parent_idxs, distances, dim, sample);
  }

  std::vector<double> goal_distances;
  for (int point_idx = 0; point_idx < points.size(); point_idx++) {

    auto distance_to_goal = get_euclidean_distance(points[point_idx], goal);
    goal_distances.push_back(distance_to_goal + distances[point_idx]);
  }

  int j = 0;
  Path path;
  // TODO: Fix to get jth nearest point
  while (j < points.size()) {
    auto nearest_pt_idx =
        std::min_element(goal_distances.begin(), goal_distances.end()) -
        goal_distances.begin();
    auto nearest = points[nearest_pt_idx];
    if (check_line_collision(nearest, goal)) {
      goal_distances[nearest_pt_idx] = std::numeric_limits<double>::max();
      j += 1;
    } else {
      path.push_back(goal);
      nearest.theta = std::atan2(goal.y - nearest.y, goal.x - nearest.x);
      path.push_back(nearest);

      auto parent_idx = parent_idxs[nearest_pt_idx];
      while (parent_idx != -1) {
        auto parent = points[parent_idx];
        parent.theta =
            std::atan2(path.back().y - parent.y, path.back().x - parent.x);
        path.push_back(parent);
        parent_idx = parent_idxs[parent_idx];
      }
      break;
    }
  }

  return Path(path.rbegin(), path.rend());
}

std::tuple<std::vector<Pose2D>, std::vector<int>, std::vector<double>>
RRTPathPlanner::sample_new_point(std::vector<Pose2D> &points,
                                 std::vector<int> &parent_idxs,
                                 std::vector<double> &distances,
                                 const Dimensions &dim, int sample) const {

  auto new_pt = create_halton_sample(sample, dim);

  // Radius in which to search for nearest points
  // TODO: Decrease dynamically based on number of points
  auto proximity =
      get_euclidean_distance({dim.x_min, dim.y_min}, {dim.x_max, dim.y_max});

  auto nearest_pt_idxs = get_nearest_neighbors(new_pt, points, proximity);

  std::vector<int> potential_parents;
  std::vector<double> potential_distances;
  std::vector<double> nearest_pt_distances;

  for (auto nearest_pt_idx : nearest_pt_idxs) {
    auto nearest_pt = points[nearest_pt_idx];
    auto dist = get_euclidean_distance(new_pt, nearest_pt);
    nearest_pt_distances.push_back(dist);

    auto new_dist = dist + distances[nearest_pt_idx];

    if (!check_line_collision(nearest_pt, new_pt)) {
      potential_parents.push_back(nearest_pt_idx);
      potential_distances.push_back(new_dist);
    }
  }

  if (potential_distances.empty()) {
    return {points, parent_idxs, distances};
  }

  auto min_idx =
      std::min_element(potential_distances.begin(), potential_distances.end()) -
      potential_distances.begin();
  auto new_dist = potential_distances[min_idx];
  auto new_parent_idx = potential_parents[min_idx];

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
    auto nearest_to_new_dist = nearest_pt_distances[j];

    if (new_dist + nearest_to_new_dist < distances[remaining_pt_idx] &&
        !check_line_collision(new_pt, remaining_pt)) {
      parent_idxs[remaining_pt_idx] = new_pt_idx;
      distances[remaining_pt_idx] = new_dist + nearest_to_new_dist;
    }
  }

  return {points, parent_idxs, distances};
}

Path RRTPathPlanner::interpolate_path(const Path &path,
                                      int desired_n_points) const {
  if (path.empty()) {
    std::cerr << "Cannot interpolate path, original path is empty."
              << std::endl;
    return Path();
  }

  int n_points = path.size();

  Path new_path;

  std::vector<double> t_path(n_points);
  for (int i = 0; i < n_points; ++i)
    t_path[i] = static_cast<double>(i) / (n_points - 1);

  for (int i = 0; i < desired_n_points; ++i) {
    double t = static_cast<double>(i) / (desired_n_points - 1);

    // Find segment index j such that t_path[j] <= t < t_path[j+1]
    int j = 0;
    while (j < n_points - 2 && t > t_path[j + 1])
      j++;

    double t0 = t_path[j];
    double t1 = t_path[j + 1];
    double alpha = (t - t0) / (t1 - t0);

    Pose2D new_pt{(1 - alpha) * path[j].x + alpha * path[j + 1].x,
                  (1 - alpha) * path[j].y + alpha * path[j + 1].y};

    new_path.push_back(new_pt);
  }

  for (int i = 0; i < new_path.size() - 1; ++i) {
    auto &pt = new_path[i];
    const auto &next_pt = new_path[i + 1];
    pt.theta = std::atan2(next_pt.y - pt.y, next_pt.x - pt.x);
  }

  return new_path;
}

VelocityProfile RRTPathPlanner::get_velocity_profile(const Path &path) const {
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

Dimensions RRTPathPlanner::get_dimensions(const Pose2D &start,
                                          const Pose2D &goal) const {
  Dimensions dim;
  dim.x_min = std::min(start.x, goal.x) - 2;
  dim.x_max = std::max(start.x, goal.x) + 2;
  dim.y_min = std::min(start.y, goal.y) - 2;
  dim.y_max = std::max(start.y, goal.y) + 2;
  return dim;
}

Pose2D RRTPathPlanner::create_halton_sample(int index,
                                            const Dimensions &dim) const {
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
      x * (dim.x_max - dim.x_min) + dim.x_min,
      y * (dim.y_max - dim.y_min) + dim.y_min,
      0.0,
  };
}

double RRTPathPlanner::get_euclidean_distance(const Pose2D &a,
                                              const Pose2D &b) const {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::vector<int> RRTPathPlanner::get_nearest_neighbors(const Pose2D &new_pt,
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

bool RRTPathPlanner::check_line_collision(const Pose2D &a,
                                          const Pose2D &b) const {
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

  for (auto obstacle : inflated_obstacles_) {
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

} // namespace wheeled_humanoid