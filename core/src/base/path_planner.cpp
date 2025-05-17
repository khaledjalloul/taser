#include "wheeled_humanoid/base/path_planner.hpp"

#include <clipper2/clipper.h>

#include <iostream>

namespace wheeled_humanoid::base {

PathPlanner::PathPlanner(int num_samples, double dt, double L)
    : num_samples_(num_samples), dt_(dt), L_(L) {}

std::vector<Obstacle>
PathPlanner::set_obstacles(const std::vector<Obstacle> obstacles) {
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

Path PathPlanner::generate_path(const Pose2D &start, const Pose2D &goal) const {
  std::vector<Pose2D> points{start};
  std::vector<int> parent_idxs{-1};
  std::vector<double> distances{0};

  Dimensions dim;
  dim.x_min = std::min({start.x, start.y, goal.x, goal.y}) - 2;
  dim.x_max = std::max({start.x, start.y, goal.x, goal.y}) + 2;
  dim.y_min = std::min({start.x, start.y, goal.x, goal.y}) - 2;
  dim.y_max = std::max({start.x, start.y, goal.x, goal.y}) + 2;

  for (int sample = 0; sample < num_samples_; sample++) {
    sample_new_point(points, parent_idxs, distances, dim, sample);
  }

  std::vector<double> goal_distances;
  for (int point_idx = 0; point_idx < points.size(); point_idx++) {
    auto distance_to_goal = get_euclidean_distance(points[point_idx], goal);
    goal_distances.push_back(distances[point_idx] + distance_to_goal);
  }

  double dubins_radius = 1;

  int j = 0;
  std::vector<DubinsSegment> dubins_path;
  // TODO: Fix to get jth nearest point
  while (j < points.size()) {
    auto nearest_pt_idx =
        std::min_element(goal_distances.begin(), goal_distances.end()) -
        goal_distances.begin();
    auto nearest = points[nearest_pt_idx];
    auto dubins_seg_to_goal = get_dubins_segment(nearest, goal, dubins_radius);
    if (dubins_seg_to_goal.collides_with(inflated_obstacles_)) {
      goal_distances[nearest_pt_idx] = std::numeric_limits<double>::max();
      j += 1;
    } else {
      dubins_path.push_back(dubins_seg_to_goal);
      auto parent_idx = parent_idxs[nearest_pt_idx];

      while (parent_idx != -1) {
        auto parent = points[parent_idx];
        auto path = get_dubins_segment(parent, nearest, dubins_radius);

        dubins_path.push_back(path);
        parent_idx = parent_idxs[parent_idx];
      }
      break;
    }
  }

  // Interpolate
  // Path path;

  return Path();
}

std::tuple<std::vector<Pose2D>, std::vector<int>, std::vector<double>>
PathPlanner::sample_new_point(std::vector<Pose2D> &points,
                              std::vector<int> &parent_idxs,
                              std::vector<double> &distances,
                              const Dimensions &dim, int sample) const {

  auto new_pt = create_halton_sample(sample, dim);

  // Radius in which to search for nearest points
  // TODO: Decrease dynamically based on number of points
  auto proximity =
      get_euclidean_distance({dim.x_min, dim.y_min}, {dim.x_max, dim.y_max});

  auto nearest_pt_idxs = get_nearest_neighbors(new_pt, points, proximity);

  double dubins_radius = 1;

  std::vector<int> potential_parents;
  std::vector<DubinsSegment> potential_dubins_segs;
  std::vector<double> potential_distances;

  for (auto nearest_pt_idx : nearest_pt_idxs) {
    auto nearest_pt = points[nearest_pt_idx];

    auto dubins_seg = get_dubins_segment(nearest_pt, new_pt, dubins_radius);

    if (!dubins_seg.collides_with(inflated_obstacles_)) {
      auto dist = dubins_seg.length();
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

  std::cout << "x: " << new_pt.x << " y: " << new_pt.y
            << " th: " << new_pt.theta << std::endl;
  std::cout << "x: " << dubins_seg.line.end.x << " y: " << dubins_seg.line.end.y
            << " th: " << dubins_seg.line.end.theta << std::endl;
  std::cout << "------------" << std::endl;

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
        get_dubins_segment(new_pt, remaining_pt, dubins_radius);
    auto nearest_to_new_dist = remaining_dubins_seg.length();

    if (new_dist + nearest_to_new_dist < distances[remaining_pt_idx] &&
        !remaining_dubins_seg.collides_with(inflated_obstacles_)) {
      points[remaining_pt_idx].theta = remaining_dubins_seg.line.end.theta;
      parent_idxs[remaining_pt_idx] = new_pt_idx;
      distances[remaining_pt_idx] = new_dist + nearest_to_new_dist;
    }
  }

  return {points, parent_idxs, distances};
}

Path PathPlanner::interpolate_path(const Path &path,
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

Pose2D PathPlanner::create_halton_sample(int index,
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