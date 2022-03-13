#include "bmb_state_estimation/OmniscientStateEstimationNode.h"
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <bmb_msgs/AircraftState.h>
#include <vector>

OmniscientStateEstimationNode::OmniscientStateEstimationNode(ros::NodeHandle& nh) {
  // initialize subscribers
  model_states_sub_ =
      nh.subscribe("/gazebo/model_states", 1,
                   &OmniscientStateEstimationNode::modelStatesCallback, this);

  // initialize publishers
  aircraft_state_pub_ =
      nh.advertise<bmb_msgs::AircraftState>("/aircraft_state", 1);
}

void OmniscientStateEstimationNode::spin() {
  ros::spin();
}

void OmniscientStateEstimationNode::modelStatesCallback(const gazebo_msgs::ModelStates& msg) {
  for (size_t i = 0; i < msg.name.size(); i++) {
    if (msg.name[i] == "aris") {
      bmb_msgs::AircraftState state;
      state.pose = msg.pose[i];
      state.twist = msg.twist[i];
      aircraft_state_pub_.publish(state);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "omnsicient_state_estimation_node");
  ros::NodeHandle nh;
  OmniscientStateEstimationNode node{nh};
  node.spin();
}
