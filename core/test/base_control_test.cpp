#include <gtest/gtest.h>

#include "wheeled_humanoid/robot.hpp"

class BaseControlTest : public ::testing::Test {
protected:
  void SetUp() override {}

  void TearDown() override {}
  wheeled_humanoid::base::Dimensions dim_{-2, 5, -2, 5};
  wheeled_humanoid::Robot robot_{0.1, 1, 1, 0.5, 10, 1, 100, dim_};
};

TEST_F(BaseControlTest, follow_path) {
  wheeled_humanoid::Pose2D goal_pose{3.0, 3.0};

  auto initial_pose = robot_.base->pose;
  auto initial_err = Eigen::Vector2d(initial_pose.x - goal_pose.x,
                                     initial_pose.y - goal_pose.y)
                         .norm();

  robot_.plan_path(goal_pose);
  for (int i = 0; i < 10; i++) {
    robot_.move_base_step();
  }

  auto final_pose = robot_.base->pose;
  auto final_err =
      Eigen::Vector2d(final_pose.x - goal_pose.x, final_pose.y - goal_pose.y)
          .norm();

  EXPECT_LE(final_err, initial_err);
}
