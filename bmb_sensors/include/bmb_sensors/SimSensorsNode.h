#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <bmb_msgs/ControlInputs.h>

class SimSensorsNode {
 public:
  SimSensorsNode(ros::NodeHandle& nh);

  void spin();

 private:
  void controlInputsCallback(const bmb_msgs::ControlInputs& msg);

  void modelStatesCallback(const gazebo_msgs::ModelStates& msg);

  bmb_msgs::ControlInputs latest_control_inputs;
  ros::Subscriber model_states_sub_;
  ros::Publisher pressure_sensor_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher gps_pub_;
  ros::Publisher rail_detection_pub_;
  ros::Publisher optical_flow_pub_;
};
