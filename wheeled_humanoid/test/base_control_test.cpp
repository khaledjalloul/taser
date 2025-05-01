#include <gtest/gtest.h>

#include "wheeled_humanoid/robot.hpp"

class BaseControlTest : public ::testing::Test {
protected:
  void SetUp() override {}

  void TearDown() override {}

  wheeled_humanoid::Robot robot_;
};

TEST_F(BaseControlTest, follow_path) {
  robot_.plan_path();

  for (int i = 0; i < robot_.planned_path.size() - robot_.base_controller.N;
       i++) {
    robot_.follow_path_step();
  }

  auto final_pose = robot_.base.pose;
  auto diff = Eigen::Vector2d(final_pose.x - robot_.planned_path.back().x,
                              final_pose.y - robot_.planned_path.back().y);
  auto err = diff.norm();

  EXPECT_LE(err, 0.3) << "Final pose: (" << final_pose.x << ", " << final_pose.y
                      << "), expected: (" << robot_.planned_path.back().x
                      << ", " << robot_.planned_path.back().y
                      << "), error: " << err;
}
