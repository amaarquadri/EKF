#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

class OmniscientStateEstimationNode {
 public:
  OmniscientStateEstimationNode(ros::NodeHandle& nh);

  void spin();

 private:
  void modelStatesCallback(const gazebo_msgs::ModelStates& msg);

  // Subscriber Objects (need to keep in scope)
  ros::Subscriber model_states_sub_;

  // Publisher Objects
  ros::Publisher aircraft_state_pub_;
};
