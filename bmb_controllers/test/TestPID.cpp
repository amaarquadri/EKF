#include <bmb_controllers/PIDFFController.h>
#include <bmb_math/TransferFunction.h>
#include <bmb_math/Vector.h>
#include <bmb_utilities/ControllerGains.h>
#include <bmb_utilities/MathUtils.h>
#include <gtest/gtest.h>
#include <cmath>
#include <ros/package.h>
#include <fstream>

TEST(TestPID, testPID) {
  TransferFunction<double, 1, 2> first_order = {1, 1, 2};
  TransferFunction<double, 1, 3> double_integrator = {1, 0, 0, 1};
  ControllerGains rollPIDGains(1, 0.2, 2);
  const double dt = 1e-3;
  const int time = 30;
  const int N = time / dt;
  const double M_PI_6 = M_PI / 6.0;

  PIDFFController<double> rollPID(rollPIDGains, dt);
  auto discrete_first_order = first_order.discretize(dt);
  auto discrete_double_integrator = double_integrator.discretize(dt);

  const double ten_degrees = 10.0 * M_PI / 180.0;
  Vector<double, N> roll_ref = ten_degrees * discrete_first_order.step<N>();
  Vector<double, N> aileron_command;
  Vector<double, N> roll_actual;

  aileron_command[0] = rollPID.update(0, roll_ref[0]);
  const double V_SQUARED = 100;
  const double AILERON_M = 0.0547822725108043;
  const double IXX_INV = 1 / 12.327;
  double ang_acceleration;
  for (int i = 1; i < N; i++) {
    ang_acceleration = bmb_utilities::saturation(
        aileron_command[i - 1] * AILERON_M * V_SQUARED * IXX_INV, M_PI_6);
    roll_actual[i] = discrete_double_integrator.next_output(ang_acceleration);
    aileron_command[i] = rollPID.update(roll_actual[i], roll_ref[i]);
  }
  const std::string directory =
      ros::package::getPath("bmb_controllers") + "/test/output/";
  std::ofstream out1(directory + "reference_roll.csv");
  ASSERT_TRUE(out1);
  roll_ref.toCSV(out1);
  out1.close();

  std::ofstream out2(directory + "aileron_command.csv");
  ASSERT_TRUE(out2);
  aileron_command.toCSV(out2);
  out2.close();

  std::ofstream out3(directory + "actual_roll.csv");
  ASSERT_TRUE(out3);
  roll_actual.toCSV(out3);
  out3.close();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
