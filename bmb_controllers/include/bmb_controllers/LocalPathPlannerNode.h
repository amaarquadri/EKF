#pragma once

#include <bmb_controllers/DubinsPath.h>
#include <bmb_controllers/PIDFFController.h>
#include <bmb_controllers/PurePursuit.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/StateCommand.h>
#include <ros/ros.h>

class LocalPathPlannerNode {
 public:
  LocalPathPlannerNode(ros::NodeHandle& nh, const double& update_frequency);

  void spin();

 private:
  void aircraftStateCallback(const bmb_msgs::AircraftState& msg);

  void referenceCommandCallback(const bmb_msgs::ReferenceCommand& msg);

  bmb_msgs::StateCommand getStateCommand();

  DubinsPath<double> path;
  bool update_dubins_path{false};
  PurePursuit<double> pursuer;
  PIDFFController<double> altitude_pid;

  const double update_frequency;
  bmb_msgs::AircraftState latest_aircraft_state;
  bmb_msgs::ReferenceCommand latest_reference_command;

  // subscribers and publishers
  ros::Subscriber aircraft_state_sub_;
  ros::Subscriber reference_command_sub_;
  ros::Publisher state_command_pub_;
};
