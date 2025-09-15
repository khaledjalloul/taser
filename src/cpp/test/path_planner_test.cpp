#include <gtest/gtest.h>

#include "taser/navigation/path_planner.hpp"

class PathPlannerTest : public ::testing::Test {
protected:
  void SetUp() override { rrt_.set_obstacles({}); }

  void TearDown() override {}

  taser::navigation::Dimensions dim_{-2, 5, -2, 5};
  taser::navigation::PathPlanner rrt_{200, 0.1, 0.5, 0.0, dim_};
};

TEST_F(PathPlannerTest, check_collision) {
  std::vector<taser::Obstacle> obstacles{{{1, 1}, {1, 2}, {2, 2}, {2, 1}}};
  rrt_.set_obstacles(obstacles);

  taser::navigation::DubinsSegment dubins_seg;

  // Diagonal
  dubins_seg.line = {{0, 0}, {3, 3}};
  EXPECT_TRUE(rrt_.check_collision(dubins_seg));

  // Horizontal
  dubins_seg.line = {{0, 2}, {2, 2}};
  EXPECT_TRUE(rrt_.check_collision(dubins_seg));

  // Inside
  dubins_seg.line = {{1, 1}, {2, 2}};
  EXPECT_TRUE(rrt_.check_collision(dubins_seg));

  // Outside
  dubins_seg.line = {{0, 0}, {0, 3}};
  EXPECT_FALSE(rrt_.check_collision(dubins_seg));
}

TEST_F(PathPlannerTest, generate_path_no_obstacles) {
  auto start = taser::Pose2D{0, 0};
  auto goal = taser::Pose2D{3, 3};
  auto path = rrt_.generate_path(start, goal);
  EXPECT_EQ(path.size(), 1);
}

TEST_F(PathPlannerTest, generate_path_with_obstacle) {
  std::vector<taser::Obstacle> obstacles{{{1, 1}, {1, 2}, {2, 2}, {2, 1}}};
  rrt_.set_obstacles(obstacles);

  auto start = taser::Pose2D{0, 0};
  auto goal = taser::Pose2D{3, 3};
  auto path = rrt_.generate_path(start, goal);
  EXPECT_GT(path.size(), 1);
}
