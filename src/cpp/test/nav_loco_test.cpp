#include <gtest/gtest.h>

#include "taser/robot.hpp"

class NavLocoTest : public ::testing::Test {
protected:
  void SetUp() override {}

  void TearDown() override {}

  int N = 10;
  taser::navigation::Dimensions dim_{-2, 5, -2, 5};
  taser::Robot robot_{{0.1, 1, 1, 0.5, N, 1, 50, dim_}};
};

TEST_F(NavLocoTest, follow_path) {
  taser::Pose2D goal_pose{3.0, 3.0};

  auto initial_pose = robot_.base->pose;
  auto initial_err = Eigen::Vector2d(initial_pose.x - goal_pose.x,
                                     initial_pose.y - goal_pose.y)
                         .norm();

  auto num_points = robot_.plan_path(goal_pose);
  for (int i = 0; i < num_points - N; i++) {
    auto res = robot_.move_base_step();
    auto err = std::get<2>(res);
    EXPECT_LE(err, initial_err);
  }

  auto final_pose = robot_.base->pose;
  auto final_err =
      Eigen::Vector2d(final_pose.x - goal_pose.x, final_pose.y - goal_pose.y)
          .norm();

  EXPECT_LE(final_err, 0.1);
}
