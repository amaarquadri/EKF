#include "bmb_state_estimation/RPYNode.h"
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/RPY.h>
#include <ros/ros.h>

RPYNode::RPYNode(ros::NodeHandle& nh) {
  // initialize subscribers
  aircraft_state_sub_ =
      nh.subscribe("/aircraft_state", 1, &RPYNode::aircraftStateCallback, this);

  // initialize publishers
  rpy_pub_ = nh.advertise<bmb_msgs::RPY>("/rpy", 1);
}

void RPYNode::spin() { ros::spin(); }

void RPYNode::aircraftStateCallback(const bmb_msgs::AircraftState& msg) {
  const Quaternion<double> quat = Quaternion<double>{msg.pose.orientation};
  bmb_msgs::RPY rpy;
  rpy.roll = quat.getRoll();
  rpy.pitch = quat.getPitch();
  rpy.yaw = quat.getYaw();
  rpy_pub_.publish(rpy);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rpy_node");
  ros::NodeHandle nh;
  RPYNode node{nh};
  node.spin();
}
