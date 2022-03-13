#include <bmb_controllers/PosVelState.h>
#include <bmb_controllers/DubinsPath.h>
#include <bmb_math/Vector.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <fstream>

TEST(TestDubinsPath, testDubinsPath) {
  const std::string directory =
      ros::package::getPath("bmb_controllers") + "/test/output/";
  using Vector2 = Vector<double, 2>;
  PosVelState start{Vector2{0, 0}, Vector2{0, 1}};
  PosVelState goal{Vector2{10, 0}, Vector2{0, -1}};
  static constexpr double radius = 1;
  DubinsPath<double> path = DubinsPath<double>::create(start, goal, radius);
  ASSERT_TRUE(path[0].isCircle() && path[0].isRightTurn() &&
              path[1].isLine() && path[2].isCircle() &&
              path[2].isRightTurn());
  ASSERT_TRUE(std::fabs((path.closestPoint(Vector2{4.5, 1}) - Vector2{4.5, 1})
                            .magnitude()) < 0.001);

  std::ofstream out1(directory + "dubins_path1.csv");
  ASSERT_TRUE(out1);
  path.toCSV(out1);
  out1.close();

  goal = PosVelState{Vector2{5, 3}, Vector2{-1, 2}};
  path = DubinsPath<double>::create(start, goal, radius);
  std::ofstream out2(directory + "dubins_path2.csv");
  ASSERT_TRUE(out2);
  path.toCSV(out2);
  out2.close();

  goal = {Vector2{0.2, 1.1}, Vector2{-0.2, -3}};
  path = DubinsPath<double>::create(start, goal, radius);
  std::ofstream out3(directory + "dubins_path3.csv");
  ASSERT_TRUE(out3);
  path.toCSV(out3);
  out3.close();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
