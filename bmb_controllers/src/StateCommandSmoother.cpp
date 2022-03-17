#include "bmb_controllers/StateCommandSmoother.h"
#include <bmb_math/TransferFunction.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_utilities/MathUtils.h>

static constexpr double MAX_ACCELERATION = 3;  // m/s^2

/**
 * Based on max tail velocity of 0.3m/s and a 1.3m distance from the rotation
 * axis to the tail.
 */
static constexpr double MAX_PITCH_RATE = 0.3 / 1.3;  // rad/s

/**
 * Based on a most aggressive command of changing from 10m/s degrees to 20m/s
 * degrees. This is a step input of 10m/s.
 * Setting the time constant to 10m/s / MAX_ACCELERATION ensures that in this
 * worst case scenario the roll rate will equal the MAX_ROLL_RATE.
 */
static const TransferFunction<double, 1, 2> SPEED_SMOOTHER{
    1, 10 / MAX_ACCELERATION, 1};

static const TransferFunction<double, 1, 2> ROLL_SMOOTHER{1, 2, 1};

/**
 * Based on a most aggressive command of changing from -7 degrees to +8 degrees.
 * This is a step input of 15 degrees.
 * Setting the time constant to 15 degrees / MAX_PITCH_RATE ensures that in this
 * worst case scenario the pitch rate will equal the MAX_PITCH_RATE.
 */
static const TransferFunction<double, 1, 2> PITCH_SMOOTHER{
    1, bmb_utilities::deg2rad(15) / MAX_PITCH_RATE, 1};

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
