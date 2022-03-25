#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>

namespace bmb_world_model {
/**
 * @return The applied forces and torques in the robot's NED reference frame
 */
// aerodynamic slope constants
static constexpr Wrench<double> BODY_M_WRENCH{
    -0.320883207023485, 0, -1.39627650007691, 0, -0.250343385330279, 0};
static constexpr Wrench<double> AILERON_M_WRENCH{
    -0.0331320958688885, 0,
    0.131535586088028,   0.0547822725108043,
    0.0125388081950779,  0.00886500122201903};
static constexpr Wrench<double> ELEVATOR_M_WRENCH{
    -0.0238646243046933, 0, 0.0803509547078008, 0, 0.0842508724650606, 0};
static constexpr Wrench<double> RUDDER_M_WRENCH{0, 0, 0,
                                                0, 0, 0.000279918236359769};

// aerodynamic offset constants
static constexpr Wrench<double> BODY_B_WRENCH{
    -0.0595573697856826, 0, -0.204721453757253, 0, 0.0015675980775375, 0};

Wrench<double> getAppliedLoads(const bmb_msgs::AircraftState& state,
                               const bmb_msgs::ControlInputs& control_inputs);

Wrench<double> wrenchFromAOA(const Vector3<double>& body_vel,
                             const double& sin_aoa_xz);

Wrench<double> wrenchFromAileron(const Vector3<double>& body_vel,
                                 const double& right_aileron_angle);

Wrench<double> wrenchFromElevator(const Vector3<double>& body_vel,
                                  const double& elevator_angle);

Wrench<double> wrenchFromRudder(const Vector3<double>& body_vel,
                                const double& sin_aoa_xy);

Matrix<double, 6, bmb_msgs::AircraftState::SIZE> getAppliedLoadsJacobian(
    const bmb_msgs::AircraftState& state,
    const bmb_msgs::ControlInputs& control_inputs);
}  // namespace bmb_world_model
