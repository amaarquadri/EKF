#include "bmb_gazebo/ARISControlPlugin.h"
#include <bmb_gazebo/Utils.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_utilities/CoordinateSystems.h>
#include <bmb_world_model/AppliedLoads.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <array>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <string>

using namespace gazebo;

ARISControlPlugin::~ARISControlPlugin() {
#if GAZEBO_MAJOR_VERSION >= 8
  this->update_connection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->update_connection);
#endif
}

void ARISControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (!_model) {
    ROS_FATAL("ARISControlPlugin _model pointer is NULL");
    return;
  }
  if (!_sdf) {
    ROS_FATAL("ARISControlPlugin _sdf pointer is NULL");
    return;
  }
  if (!ros::isInitialized()) {
    ROS_FATAL("ROS API plugin not loaded");
    return;
  }

  // Read the required joint name parameters.
  static const std::array<std::string, 4> required_params{
      // cannot use constexpr with std::string
      "propeller", "right_aileron", "left_aileron", "elevator"};
  for (size_t i = 0; i < required_params.size(); i++) {
    const std::string& param = required_params[i];
    if (!_sdf->HasElement(param)) {
      ROS_FATAL_STREAM("Unable to find the <" << param << "> parameter.");
      return;
    }

    const std::string joint_name = _sdf->Get<std::string>(param);
    this->joints[i] = _model->GetJoint(joint_name);
    if (!this->joints[i]) {
      ROS_FATAL_STREAM("Failed to find joint [" << joint_name
                                                << "] aborting plugin load.");
      return;
    }
  }

  if (!_sdf->HasElement("base_link")) {
    ROS_FATAL_STREAM("Unable to find the <base_link> parameter.");
    return;
  }

  const std::string base_link_name = _sdf->Get<std::string>("base_link");
  base_link = _model->GetLink(base_link_name);
  if (!base_link) {
    ROS_FATAL_STREAM("Failed to find link [" << base_link_name
                                             << "] aborting plugin load.");
    return;
  }

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
      [&](const common::UpdateInfo& /** update_info **/) { this->update(); });

  // Initialize ROS subscriber
  this->control_inputs_sub = this->nh.subscribe(
      "/control_inputs", 1, &ARISControlPlugin::controlInputsCallback, this);

  // initialize linear velocity to 10m/s
  base_link->SetLinearVel(bmbToIgnitionVector3(Vector3<double>{10}));

  // set camera to follow the model
  // TODO: get this working
  gui::Events::follow(_model->GetName());

  ROS_INFO("ARIS ready to fly. The force will be with you");
}

void ARISControlPlugin::controlInputsCallback(
    const bmb_msgs::ControlInputs& msg) {
  std::lock_guard<std::mutex> lock(this->mutex);
  this->latest_control_inputs = msg;
}

bmb_msgs::AircraftState ARISControlPlugin::getAircraftState() const {
  bmb_msgs::AircraftState state;
  const auto pose = base_link->WorldCoGPose();
  // TODO: check coordinate system transformation
  ignitionToGeometryVector3(pose.Pos(), state.pose.position);
  ignitionToGeometryQuaternion(pose.Rot(), state.pose.orientation);
  ignitionToGeometryVector3(base_link->RelativeLinearVel(), state.twist.linear);
  ignitionToGeometryVector3(base_link->RelativeAngularVel(),
                            state.twist.angular);
  return state;
}

void ARISControlPlugin::update() {
  // 9000RPM is max speed and 2.21kg is max force
  static constexpr double PROPELLER_FORCE_TO_VEL_RATIO =
      (9000 * 60 / (2 * M_PI)) / (2.21 * 9.81);

  // read control_inputs in a thread-safe way
  bmb_msgs::ControlInputs control_inputs;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    control_inputs = this->latest_control_inputs;
  }

  // apply loads
  const Wrench<double> wrench = bmb_utilities::NEDToNWU(
      getAppliedLoads(getAircraftState(), control_inputs));
  base_link->AddRelativeForce(bmbToIgnitionVector3(wrench.force));
  base_link->AddRelativeTorque(bmbToIgnitionVector3(wrench.torque));

  // geometric effects of the propeller and control surfaces
  this->joints[kPropeller]->SetVelocity(
      0, PROPELLER_FORCE_TO_VEL_RATIO * control_inputs.propeller_force);
#if GAZEBO_MAJOR_VERSION >= 8
  this->joints[kRightAileron]->SetPosition(0,
                                           control_inputs.right_aileron_angle);
  this->joints[kLeftAileron]->SetPosition(0,
                                          -control_inputs.right_aileron_angle);
  this->joints[kElevator]->SetPosition(0, control_inputs.elevator_angle);
#else
  this->joints[kRightAileron]->SetAngle(0, control_inputs.right_aileron_angle);
  this->joints[kLeftAileron]->SetAngle(0, -control_inputs.right_aileron_angle);
  this->joints[kElevator]->SetAngle(0, control_inputs.elevator_angle);
#endif
}

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(ARISControlPlugin);
