#include "bmb_controllers/StateCommandSmoother.h"
#include <bmb_math/TransferFunction.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_utilities/MathUtils.h>

static const TransferFunction<double, 1, 2> SPEED_SMOOTHER{1, 1, 0.1};

static const TransferFunction<double, 1, 2> ROLL_SMOOTHER{1, 1, 2};

static const TransferFunction<double, 1, 2> PITCH_SMOOTHER{1, 1, 2};

StateCommandSmoother::StateCommandSmoother(const double& update_frequency) {
  const double dt = 1 / update_frequency;
  speed_smoother = SPEED_SMOOTHER.discretize(dt);
  roll_smoother = ROLL_SMOOTHER.discretize(dt);
  pitch_smoother = PITCH_SMOOTHER.discretize(dt);
}

bmb_msgs::StateCommand StateCommandSmoother::getSmoothedStateCommand(
    const bmb_msgs::StateCommand& state_command) {
  bmb_msgs::StateCommand smoothed_command;
  smoothed_command.speed = speed_smoother.next_output(state_command.speed);
  smoothed_command.roll = roll_smoother.next_output(state_command.roll);
  smoothed_command.pitch = roll_smoother.next_output(state_command.pitch);
  return smoothed_command;
}
