#include "bmb_controllers/LowLevelControlLoopNode.h"
#include <bmb_controllers/PIDFFController.h>
#include <bmb_controllers/StateCommandSmoother.h>
#include <bmb_math/Quaternion.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/StateCommand.h>
#include <bmb_utilities/ControllerGains.h>
#include <bmb_world_model/Constants.h>
#include <algorithm>
#include <cmath>

static constexpr double MAX_PROPELLER_FORCE = 2.2 * 9.81;
static constexpr double MAX_AILERON_ANGLE = M_PI / 6;
static constexpr double MAX_ELEVATOR_ANGLE = M_PI / 6;

LowLevelControlLoopNode::LowLevelControlLoopNode(ros::NodeHandle& nh,
                                                 const double& update_frequency)
    : update_frequency(update_frequency),
      smoother(StateCommandSmoother{update_frequency}) {
  const double update_period = 1 / update_frequency;
  speed_pid = PIDFFController<double>{THROTTLE_GAIN, update_period};
  roll_pid = PIDFFController<double>{ROLL_GAIN, update_period};
  pitch_pid = PIDFFController<double>{PITCH_GAIN, update_period};

  // initialize subscribers
  aircraft_state_sub_ =
      nh.subscribe("aircraft_state", 1,
                   &LowLevelControlLoopNode::aircraftStateCallback, this);
  state_command_sub_ = nh.subscribe(
      "state_command", 1, &LowLevelControlLoopNode::stateCommandCallback, this);

  // initialize publishers
  control_inputs_pub_ =
      nh.advertise<bmb_msgs::ControlInputs>("control_inputs", 1);
}

bmb_msgs::ControlInputs LowLevelControlLoopNode::getControlInputs() {
  const bmb_msgs::StateCommand smoothed_command =
      smoother.getSmoothedStateCommand(latest_state_command);

  const Quaternion<double> orientation{latest_aircraft_state.pose.orientation};
  const double pitch = orientation.getPitch();
  const double roll = orientation.getRoll();

  bmb_msgs::ControlInputs control_inputs{};
  control_inputs.propeller_force =
      std::clamp(speed_pid.update(latest_aircraft_state.twist.linear.x,
                                  smoothed_command.speed),
                 0, MAX_PROPELLER_FORCE);
  control_inputs.right_aileron_angle =
      std::clamp(roll_pid.update(roll, smoothed_command.roll),
                 -MAX_AILERON_ANGLE, MAX_AILERON_ANGLE);
  control_inputs.elevator_angle =
      std::clamp(pitch_pid.update(pitch, smoothed_command.pitch),
                 -MAX_ELEVATOR_ANGLE, MAX_ELEVATOR_ANGLE);
  return control_inputs;
}

void LowLevelControlLoopNode::aircraftStateCallback(
    const bmb_msgs::AircraftState& msg) {
  latest_aircraft_state = msg;
}

void LowLevelControlLoopNode::stateCommandCallback(
    const bmb_msgs::StateCommand& msg) {
  latest_state_command = msg;
}

void LowLevelControlLoopNode::spin() {
  ros::Rate rate{update_frequency};
  while (ros::ok()) {
    ros::spinOnce();
    control_inputs_pub_.publish(getControlInputs());
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "low_level_control_loop_node");
  ros::NodeHandle nh;
  LowLevelControlLoopNode node{nh, 100};
  node.spin();
}
