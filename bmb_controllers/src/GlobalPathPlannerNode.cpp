#include "bmb_controllers/GlobalPathPlannerNode.h"
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_utilities/MathUtils.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <cassert>
#include <fstream>
#include <optional>
#include <vector>

GlobalPathPlannerNode::GlobalPathPlannerNode(ros::NodeHandle& nh)
    : waypoint_index(0) {
  aircraft_state_sub_ = nh.subscribe(
      "aircraft_state", 1, &GlobalPathPlannerNode::aircraftStateCallback, this);

  reference_command_pub_ =
      nh.advertise<bmb_msgs::ReferenceCommand>("reference_command", 1);

  // TODO: implement hard coded coordinates csv
  std::ifstream hard_coded_coordinates{
      ros::package::getPath("bmb_controllers") +
      "/config/reference_trajectory_loop.csv"};
  assert(hard_coded_coordinates);
  std::string data;
  while (std::getline(hard_coded_coordinates, data)) {
    std::stringstream sep(data);
    std::string field;
    bmb_msgs::ReferenceCommand command;

    // First column is x_pos
    std::getline(sep, field, ',');
    command.x_pos = std::stod(field);

    // Second column is y_pos
    std::getline(sep, field, ',');
    command.y_pos = std::stod(field);

    // Third column is x_vel
    std::getline(sep, field, ',');
    command.x_vel = std::stod(field);

    // Fourth column is y_vel
    std::getline(sep, field, ',');
    command.y_vel = std::stod(field);

    // Fifth column is altitude
    std::getline(sep, field, ',');
    command.altitude = std::stod(field);

    reference_commands.push_back(command);
  }
  hard_coded_coordinates.close();
}

void GlobalPathPlannerNode::aircraftStateCallback(
    const bmb_msgs::AircraftState& msg) {
  using bmb_utilities::squared;
  static constexpr double RADIUS_TOL_SQUARED =
      bmb_utilities::squared(RADIUS_TOL);
  const bmb_msgs::ReferenceCommand& command =
      reference_commands[waypoint_index];
  if (squared(msg.pose.position.x - command.x_pos) +
          squared(msg.pose.position.y - command.y_pos) <
      RADIUS_TOL_SQUARED) {
    waypoint_index = bmb_utilities::positiveModulus(++waypoint_index,
                                                    reference_commands.size());
    reference_command_pub_.publish(reference_commands[waypoint_index]);
  }
}

void GlobalPathPlannerNode::spin() { ros::spin(); }

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_path_planner_node");
  ros::NodeHandle nh;
  GlobalPathPlannerNode node{nh};
  node.spin();
}
