#pragma once

#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <array>
#include <cstddef>
#include <mutex>

namespace gazebo {

class ARISControlPlugin : public ModelPlugin {
 public:
  ARISControlPlugin() = default;

  ~ARISControlPlugin();

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

 private:
  void controlInputsCallback(const bmb_msgs::ControlInputs& msg);

  bmb_msgs::AircraftState getAircraftState() const;

  void update();

  static constexpr size_t kPropeller = 0;
  static constexpr size_t kRightAileron = 1;
  static constexpr size_t kLeftAileron = 2;
  static constexpr size_t kElevator = 3;

  std::array<physics::JointPtr, 4> joints;
  physics::LinkPtr base_link;
  event::ConnectionPtr update_connection;
  // we need to run a ROS node from this Gazebo plugin in order to read from ROS
  // topics
  ros::NodeHandle nh;
  ros::Subscriber control_inputs_sub;
  bmb_msgs::ControlInputs latest_control_inputs;
  std::mutex mutex;
};

}  // namespace gazebo
