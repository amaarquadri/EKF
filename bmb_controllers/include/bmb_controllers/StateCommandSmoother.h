#pragma once

#include <bmb_math/TransferFunction.h>
#include <bmb_msgs/StateCommand.h>

class StateCommandSmoother {
 public:
  StateCommandSmoother(const double& dt);

  bmb_msgs::StateCommand getSmoothedStateCommand(
      const bmb_msgs::StateCommand& state_command);

 private:
  TransferFunction<double, 2, 2> speed_smoother;
  TransferFunction<double, 2, 2> roll_smoother;
  TransferFunction<double, 2, 2> pitch_smoother;
};
