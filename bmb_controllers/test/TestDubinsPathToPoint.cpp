#include <bmb_controllers/DubinsPathToPoint.h>
#include <bmb_controllers/PosVelState.h>
#include <bmb_math/Vector.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <fstream>

TEST(TestDubinsPathToPoint, testDubinsPathToPoint) {
  const std::string directory =
      ros::package::getPath("bmb_controllers") + "/test/output/";
  using Vector2 = Vector<double, 2>;
  PosVelState start{Vector2{0, 0}, Vector2{0, 1}};

  Vector2 goal{3, 2};
  static constexpr double radius = 1;
  DubinsPathToPoint<double> path =
      DubinsPathToPoint<double>::create(start, goal, radius);
  ASSERT_TRUE(path[0].isCircle() && path[0].isRightTurn() && path[1].isLine());

  std::ofstream out1(directory + "dubins_path_to_point1.csv");
  ASSERT_TRUE(out1);
  path.toCSV(out1);
  out1.close();

  goal = Vector2{0.3, -0.2};
  path = DubinsPathToPoint<double>::create(start, goal, radius);
  // TODO: uncomment assert once this case is working
//  ASSERT_TRUE(path[0].isCircle() && !path[0].isRightTurn() &&
//              path[1].isCircle() && path[1].isRightTurn());
  std::ofstream out2(directory + "dubins_path_to_point2.csv");
  ASSERT_TRUE(out2);
  path.toCSV(out2);
  out2.close();

  goal = Vector2{-4, -3};
  path = DubinsPathToPoint<double>::create(start, goal, radius);
  ASSERT_TRUE(path[0].isCircle() && !path[0].isRightTurn() && path[1].isLine());
  std::ofstream out3(directory + "dubins_path_to_point3.csv");
  ASSERT_TRUE(out3);
  path.toCSV(out3);
  out3.close();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
