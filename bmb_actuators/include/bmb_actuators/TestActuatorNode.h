#pragma once

#include <bmb_msgs/ControlInputs.h>
#include <ros/ros.h>

class TestActuatorNode {
 public:
  TestActuatorNode(ros::NodeHandle& nh, const double& update_frequency);

  void spin();

 private:
  ros::Publisher control_inputs_pub_;
};
