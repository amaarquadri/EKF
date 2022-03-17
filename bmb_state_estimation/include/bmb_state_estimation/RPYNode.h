#pragma once

#include <bmb_msgs/AircraftState.h>
#include <ros/ros.h>

class RPYNode {
 public:
  RPYNode(ros::NodeHandle& nh);

  void spin();

 private:
  void aircraftStateCallback(const bmb_msgs::AircraftState& msg);

  // Subscriber Objects (need to keep in scope)
  ros::Subscriber aircraft_state_sub_;

  // Publisher Objects
  ros::Publisher rpy_pub_;
};
