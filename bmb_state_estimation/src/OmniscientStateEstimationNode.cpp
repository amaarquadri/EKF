#include "bmb_state_estimation/OmniscientStateEstimationNode.h"
#include <bmb_msgs/AircraftState.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <vector>

OmniscientStateEstimationNode::OmniscientStateEstimationNode(
    ros::NodeHandle& nh) {
  // initialize subscribers
  model_states_sub_ =
      nh.subscribe("/gazebo/model_states", 1,
                   &OmniscientStateEstimationNode::modelStatesCallback, this);

  // initialize publishers
  aircraft_state_pub_ =
      nh.advertise<bmb_msgs::AircraftState>("/aircraft_state", 1);
}

void OmniscientStateEstimationNode::spin() { ros::spin(); }

void OmniscientStateEstimationNode::modelStatesCallback(
    const gazebo_msgs::ModelStates& msg) {
  for (size_t i = 0; i < msg.name.size(); i++) {
    if (msg.name[i] == "aris") {
      const auto& pose = msg.pose[i];
      const auto& twist = msg.twist[i];

      bmb_msgs::AircraftState state;
      bmb_utilities::NWUtoNED(Vector3<double>{pose.position})
          .copyTo(state.pose.position);
      bmb_utilities::NWUtoNED(Quaternion<double>{pose.orientation})
          .copyTo(state.pose.orientation);
      bmb_utilities::NWUtoNED(Vector3<double>{twist.linear})
          .copyTo(state.twist.linear);
      bmb_utilities::NWUtoNED(Vector3<double>{twist.angular})
          .copyTo(state.twist.angular);
      aircraft_state_pub_.publish(state);
      return;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "omniscient_state_estimation_node");
  ros::NodeHandle nh;
  OmniscientStateEstimationNode node{nh};
  node.spin();
}
