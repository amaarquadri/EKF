#include <bmb_controllers/PosVelState.h>
#include <bmb_controllers/DubinsPath.h>
#include <bmb_controllers/PurePursuit.h>
#include <bmb_math/Vector.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <fstream>

TEST(TestPurePursuit, testPurePursuit) {
  const std::string directory =
      ros::package::getPath("bmb_controllers") + "/test/output/";
  using Vector2 = Vector<double, 2>;
  PosVelState start = {Vector2{0, 0}, Vector2{0, 1}};
  PosVelState goal = {Vector2{5, 3}, Vector2{-1, 2}};
  static constexpr double min_dubins_radius = 5;
  DubinsPath<double> path =
      DubinsPath<double>::create(start, goal, min_dubins_radius);
  PurePursuit<double> pursuer{2, 7, path};

  std::ofstream out1(directory + "pure_pursuit1.csv");
  ASSERT_TRUE(out1);
  pursuer.toCSV(out1, start);
  out1.close();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
