#include <gtest/gtest.h>

#include "wheeled_humanoid/base/path_planner.hpp"

class PathPlannerTest : public ::testing::Test {
protected:
  void SetUp() override { rrt_.set_obstacles({}); }

  void TearDown() override {}

  wheeled_humanoid::base::PathPlanner rrt_{200, 0.1, 0.5};
};

TEST_F(PathPlannerTest, check_collision) {
  std::vector<wheeled_humanoid::Obstacle> obstacles{
      {{1, 1}, {1, 2}, {2, 2}, {2, 1}}};
  rrt_.set_obstacles(obstacles);

  wheeled_humanoid::base::Line diagonal{{0, 0}, {3, 3}};
  EXPECT_TRUE(diagonal.collides_with(obstacles));

  wheeled_humanoid::base::Line horizontal{{0, 2}, {2, 2}};
  EXPECT_TRUE(horizontal.collides_with(obstacles));

  wheeled_humanoid::base::Line vertical{{0, 0}, {0, 3}};
  EXPECT_FALSE(vertical.collides_with(obstacles));
}

TEST_F(PathPlannerTest, generate_path_no_obstacles) {
  auto start = wheeled_humanoid::Pose2D{0, 0};
  auto goal = wheeled_humanoid::Pose2D{3, 3};
  auto path = rrt_.generate_path(start, goal);
  EXPECT_EQ(path.size(), 2);
}

TEST_F(PathPlannerTest, generate_path_with_obstacle) {
  std::vector<wheeled_humanoid::Obstacle> obstacles{
      {{1, 1}, {1, 2}, {2, 2}, {2, 1}}};
  rrt_.set_obstacles(obstacles);

  auto start = wheeled_humanoid::Pose2D{0, 0};
  auto goal = wheeled_humanoid::Pose2D{3, 3};
  auto path = rrt_.generate_path(start, goal);
  EXPECT_GT(path.size(), 2);
}
