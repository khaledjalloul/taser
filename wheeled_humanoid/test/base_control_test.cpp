#include <gtest/gtest.h>

#include "wheeled_humanoid/robot.hpp"

class BaseControlTest : public ::testing::Test {
protected:
  void SetUp() override {}

  void TearDown() override {}

  wheeled_humanoid::Robot robot_;
};

TEST_F(BaseControlTest, follow_path) {
  wheeled_humanoid::Path desired_path{
      {0, 0, 0}, {1, 1, 0}, {2, 2, 0}, {3, 1, 0}, {4, 1, 0}};

  robot_.follow_path(desired_path);

  auto final_pose = robot_.base.pose;
  auto diff = Eigen::Vector2d(final_pose.x - desired_path.back().x,
                              final_pose.y - desired_path.back().y);
  auto err = diff.norm();

  EXPECT_LE(err, 0.3) << "Final pose: (" << final_pose.x << ", " << final_pose.y
                      << "), expected: (" << desired_path.back().x << ", "
                      << desired_path.back().y << "), error: " << err;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}