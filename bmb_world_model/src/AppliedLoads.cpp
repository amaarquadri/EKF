#include "bmb_world_model/AppliedLoads.h"
#include <bmb_differentiation/runtime/Constant.h>
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_utilities/MathUtils.h>
#include <bmb_world_model/Constants.h>
#include <cmath>

// aerodynamic slope constants
static const Wrench<double> BODY_M_WRENCH{
    1.39627650007691, 0.320883207023485, 0, 0, -0.250343385330279, 0};
static const Wrench<double> AILERON_M_WRENCH{
    -0.0022957290598182,   -0.000578264161264832, 0,
    -0.000956131027304632, 0.000218843487756994,  -0.00015472345999439};
static const Wrench<double> ELEVATOR_M_WRENCH{
    -0.00140238887889279, 0.000416516268313824, 0, 0, 0.00147045512221362, 0};
static const Wrench<double> RUDDER_M_WRENCH{0, 0, 0,
                                            0, 0, 0.000279918236359769};

// aerodynamic offset constants
static const Wrench<double> BODY_B_WRENCH{
    0.204721453757253, 0.0595573697856826, 0, 0, 0.0015675980775375, 0};

// propeller constants
static constexpr double THRUST_TORQUE_RATIO_PROPELLER = 1;
static const Vector3<double> L_FRONT_PROPELLER{0, 0, 0};

using bmb_utilities::saturation;

static Matrix<ExprPtr, 3, 4> getQuatToWeightJacExpr() {
  Matrix<ExprPtr, 3, 4> expr;

  const Quaternion<ExprPtr> quat{Variable::make("q0"), Variable::make("q1"),
                                 Variable::make("q2"), Variable::make("q3")};
  const Vector3<ExprPtr> weight =
      quat.rotate(WEIGHT.applyFunc<ExprPtr>(&Constant::make));
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 4; j++)
      expr[i][j] =
          weight[i]
              ->diff(
                  std::static_pointer_cast<Variable>(quat[j])->getIdentifier())
              ->simplify();

  return expr;
}

static const Matrix<ExprPtr, 3, 4> QUAT_TO_WEIGHT_JAC_EXPR =
    getQuatToWeightJacExpr();  // NOLINT(cert-err58-cpp)

static Wrench<double> getPropellerLoads(const double& propeller_force) {
  const Vector3<double> thrust{propeller_force};
  const Vector3<double> torque{THRUST_TORQUE_RATIO_PROPELLER * propeller_force};
  return {thrust, L_FRONT_PROPELLER.cross(thrust) + torque};
}

static Wrench<double> getGravitationalLoads(const Quaternion<double>& quat) {
  return {quat.rotate(WEIGHT), Vector3<double>{}};
}

Wrench<double> getAppliedLoads(const bmb_msgs::AircraftState& state,
                               const bmb_msgs::ControlInputs& control_inputs) {
  const auto& b_vel = state.twist.linear;
  const Quaternion<double> quat{state.pose.orientation};

  const double vx_squared = b_vel.x * b_vel.x;
  const double speed_xz_squared = vx_squared + b_vel.z * b_vel.z;
  const double speed_xy_squared = vx_squared + b_vel.y * b_vel.y;
  const double sin_aoa_xz = b_vel.z / std::sqrt(speed_xz_squared);
  const double sin_aoa_xy = -b_vel.y / std::sqrt(speed_xy_squared);

  // absolute value of aileron angle is used for the force models
  const double right_aileron_angle_mag =
      std::fabs(control_inputs.right_aileron_angle);
  const Wrench<double> aileron_wrench{right_aileron_angle_mag,
                                      right_aileron_angle_mag,
                                      right_aileron_angle_mag,
                                      right_aileron_angle_mag,
                                      control_inputs.right_aileron_angle,
                                      control_inputs.right_aileron_angle};

  // absolute value of aileron angle is used for the force models
  const double elevator_angle_mag = std::fabs(control_inputs.elevator_angle);
  const Wrench<double> elevator_wrench{
      control_inputs.elevator_angle, elevator_angle_mag,
      control_inputs.elevator_angle, control_inputs.elevator_angle,
      control_inputs.elevator_angle, control_inputs.elevator_angle};

  const Wrench<double> body_loads =
      (BODY_M_WRENCH * sin_aoa_xz + BODY_B_WRENCH) * speed_xz_squared;
  const Wrench<double> aileron_loads =
      AILERON_M_WRENCH * aileron_wrench * speed_xz_squared;
  const Wrench<double> elevator_loads =
      ELEVATOR_M_WRENCH * elevator_wrench * speed_xz_squared;
  const Wrench<double> rudder_loads =
      RUDDER_M_WRENCH * sin_aoa_xy * speed_xy_squared;

  return body_loads + aileron_loads + elevator_loads + rudder_loads +
         getPropellerLoads(control_inputs.propeller_force) +
         getGravitationalLoads(quat);
}

Matrix<double, 6, bmb_msgs::AircraftState::SIZE> getAppliedLoadsJacobian(
    const bmb_msgs::AircraftState& state,
    const bmb_msgs::ControlInputs& control_inputs) {
  auto wrench_jac = Matrix<double, 6, bmb_msgs::AircraftState::SIZE>::zeros();
  // propeller loads does not contribute since it does not depend on state
  // TODO: implement

  return wrench_jac;
}
