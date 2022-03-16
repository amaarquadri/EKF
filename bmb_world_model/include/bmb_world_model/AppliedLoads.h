#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>


namespace bmb_world_model {
/**
 * @return The applied forces and torques in the robot's NED reference frame
 */
    Wrench<double> getAppliedLoads(const bmb_msgs::AircraftState &state,
                                   const bmb_msgs::ControlInputs &control_inputs);

    const Wrench<double>wrenchFromAOA(const bmb_msgs::AircraftState &state);

    const wrench<double>
    wrenchFromAileron(const bmb_msgs::AircraftState &state, const bmb_msgs::ControlInputs &control_inputs);

    const wrench<double>
    wrenchFromElevator(const bmb_msgs::AircraftState &state, const bmb_msgs::ControlInputs &control_inputs);

    const wrench<double> wrenchFromRudder(const bmb_msgs::AircraftState &state);

    Matrix<double, 6, bmb_msgs::AircraftState::SIZE> bmb_world_model::getAppliedLoadsJacobian(
            const bmb_msgs::AircraftState &state,
            const bmb_msgs::ControlInputs &control_inputs);
}
