#include "bmb_sensors/SimSensorsNode.h"
#include <bmb_math/Accel.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/OpticalFlowReading.h>
#include <bmb_msgs/RailDetection.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_world_model/AppliedLoads.h>
#include <bmb_world_model/SensorModels.h>
#include <bmb_world_model/WrenchUtilities.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>

SimSensorsNode::SimSensorsNode(ros::NodeHandle& nh) {
  // initialize subscribers
  model_states_sub_ = nh.subscribe("/gazebo/model_states", 1,
                                   &SimSensorsNode::modelStatesCallback, this);

  // initialize publishers
  pressure_sensor_pub_ =
      nh.advertise<sensor_msgs::FluidPressure>("/pressure_sensor_reading", 1);
  imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_reading", 1);
  gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/gps_reading", 1);
  rail_detection_pub_ =
      nh.advertise<bmb_msgs::RailDetection>("/rail_detection", 1);
  optical_flow_pub_ =
      nh.advertise<bmb_msgs::OpticalFlowReading>("/optical_flow_reading", 1);
}

void SimSensorsNode::spin() { ros::spin(); }

void SimSensorsNode::controlInputsCallback(const bmb_msgs::ControlInputs& msg) {
  latest_control_inputs = msg;
}

void SimSensorsNode::modelStatesCallback(const gazebo_msgs::ModelStates& msg) {
  for (size_t i = 0; i < msg.name.size(); i++) {
    if (msg.name[i] == "aris") {
      bmb_msgs::AircraftState state;
      state.pose = msg.pose[i];
      state.twist = msg.twist[i];

      const Wrench<double> wrench =
          getAppliedLoads(state, latest_control_inputs);
      const Accel<double> accel = bmb_world_model::toAccel(wrench);
      const bmb_msgs::SensorMeasurements measurements = getSensorMeasurements(
          state, Vector3<double>{}, Vector3<double>{}, accel);

      pressure_sensor_pub_.publish(measurements.pressure_reading);
      imu_pub_.publish(measurements.imu_reading);
      gps_pub_.publish(measurements.gps_reading);
      rail_detection_pub_.publish(measurements.rail_detection);
      optical_flow_pub_.publish(measurements.optical_flow_reading);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_sensors_node");
  ros::NodeHandle nh;
  SimSensorsNode node{nh};
  node.spin();
}
