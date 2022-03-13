#pragma once

#include <bmb_controllers/PIDFFController.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/StateCommand.h>
#include <ros/ros.h>

class LowLevelControlLoopNode {
 public:
  LowLevelControlLoopNode(ros::NodeHandle& nh, const double& update_frequency);

  void spin();

 private:
  bmb_msgs::ControlInputs getControlInputs();

  void aircraftStateCallback(const bmb_msgs::AircraftState& msg);

  void stateCommandCallback(const bmb_msgs::StateCommand& msg);

  ros::Subscriber aircraft_state_sub_;
  ros::Subscriber state_command_sub_;
  ros::Publisher control_inputs_pub_;

  bmb_msgs::AircraftState latest_aircraft_state;
  bmb_msgs::StateCommand latest_state_command;
  const double update_frequency;
  PIDFFController<double> speed_pid;
  PIDFFController<double> roll_pid;
  PIDFFController<double> pitch_pid;
};
