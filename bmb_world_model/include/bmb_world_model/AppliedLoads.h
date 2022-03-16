#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>

namespace bmb_world_model {
/**
 * @return The applied forces and torques in the robot's NED reference frame
 */
Wrench<double> getAppliedLoads(const bmb_msgs::AircraftState& state,
                               const bmb_msgs::ControlInputs& control_inputs);

const Wrench<double> wrenchFromAOA(const double& body_vel,
                                   const double& sin_aoa_xz);

Wrench<double> wrenchFromAileron(const double& body_vel,
                                 const double& right_aileron_angle);

Wrench<double> wrenchFromElevator(const double& body_vel,
                                  const double& elevator_angle);

Wrench<double> wrenchFromRudder(const double& body_vel,
                                const double& sin_aoa_xy);

Matrix<double, 6, bmb_msgs::AircraftState::SIZE>
bmb_world_model::getAppliedLoadsJacobian(
    const bmb_msgs::AircraftState& state,
    const bmb_msgs::ControlInputs& control_inputs);
}  // namespace bmb_world_model
