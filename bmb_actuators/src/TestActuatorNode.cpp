#include "bmb_actuators/TestActuatorNode.h"
#include <bmb_msgs/ControlInputs.h>
#include <ros/ros.h>
#include <cmath>

TestActuatorNode::TestActuatorNode(ros::NodeHandle& nh,
                                   const double& update_frequency)
    : update_frequency(update_frequency) {
  control_inputs_pub_ =
      nh.advertise<bmb_msgs::ControlInputs>("control_inputs", 1);
}

static bmb_msgs::ControlInputs getControlInputs() {
  static constexpr double period = 2;                 // s
  static constexpr double omega = 2 * M_PI / period;  // rad/s
  const double t = ros::Time::now().toSec();
  bmb_msgs::ControlInputs msg;
  msg.propeller_force = 5;
  msg.right_aileron_angle = 0.5 * std::sin(omega * t);
  msg.elevator_angle = 0.5 * std::sin(omega * t + M_PI);
  return msg;
}

void TestActuatorNode::spin() {
  ros::Rate rate{update_frequency};
  while (ros::ok()) {
    control_inputs_pub_.publish(getControlInputs());
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_actuator_node");
  ros::NodeHandle nh;
  TestActuatorNode node{nh, 100};
  node.spin();
}
