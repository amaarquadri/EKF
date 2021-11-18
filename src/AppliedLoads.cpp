#include "AppliedLoads.h"
#include "KF.h"
#include "Constants.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Variable.h"
#include "Constant.h"
#include "math_utils.h"
#include <cmath>

#ifndef M_PI_4
// this is needed for the code to compile in the Simulink S Function Builder
#define M_PI_4 0.78539816339744830962
#endif

static Matrix<3, 4, ExprPtr> getQuatToWeightJacExpr() {
    Matrix<3, 4, ExprPtr> expr;

    Quaternion<ExprPtr> quat{Variable::make("q0"),
                             Variable::make("q1"),
                             Variable::make("q2"),
                             Variable::make("q3")};
    Vector3<ExprPtr> weight = quat.rotate(WEIGHT.applyFunc<ExprPtr>(&Constant::make));
    for (size_t i = 0; i < 3; i++) for (size_t j = 0; j < 4; j++)
        expr[i][j] = weight[i]->diff(std::static_pointer_cast<Variable>(quat[j])->getIdentifier())->simplify();

    return expr;
}

const Matrix<3, 4, ExprPtr> AppliedLoads::QUAT_TO_WEIGHT_JAC_EXPR = getQuatToWeightJacExpr(); // NOLINT(cert-err58-cpp)

void AppliedLoads::update(const ControlInputs &control_inputs) {
    last_propeller_ang_vel = getPropellerAngVelocity();
    last_control_inputs = current_control_inputs;
    current_control_inputs = control_inputs;
}

double AppliedLoads::getPropellerAngVelocity() const {
    double propeller_voltage_sat = utils::saturation<double>(current_control_inputs.propeller_voltage, 0, 12);
    double last_propeller_voltage_sat = utils::saturation<double>(last_control_inputs.propeller_voltage, 0, 12);
    return (1 / (2 * TAU_PROPELLER + T_SAMPLE)) *
           ((2 * TAU_PROPELLER - T_SAMPLE) * last_propeller_ang_vel +
            K_PROPELLER * T_SAMPLE * (last_propeller_voltage_sat + propeller_voltage_sat));
}

Wrench<> AppliedLoads::getPropellerLoads() const {
    double propeller_ang_vel = getPropellerAngVelocity();
    Vector3<> thrust{THRUST_GAIN_PROPELLER * propeller_ang_vel * propeller_ang_vel, 0, 0};
    Vector3<> torque{TORQUE_GAIN_PROPELLER * propeller_ang_vel * propeller_ang_vel, 0, 0};

    return {thrust, L_FRONT_PROPELLER.cross(thrust) + torque};
}

Wrench<> AppliedLoads::getRightAileronLoads(const double &velocity) const {
    double lift = LIFT_GAIN_AILERON * utils::saturation<double>(current_control_inputs.right_aileron_angle, M_PI_4) * velocity * velocity;
    Vector3<> force{0, 0, -lift};
    return {force, L_RIGHT_AILERON.cross(force)};
}

Wrench<> AppliedLoads::getLeftAileronLoads(const double &velocity) const {
    double lift = LIFT_GAIN_AILERON * utils::saturation<double>(current_control_inputs.left_aileron_angle, M_PI_4) * velocity * velocity;
    Vector3<> force{0, 0, -lift};
    return {force, L_LEFT_AILERON.cross(force)};
}

Wrench<> AppliedLoads::getElevatorLoads(const double &velocity) const {
    double lift = LIFT_GAIN_ELEVATOR * utils::saturation<double>(current_control_inputs.elevator_angle, M_PI_4) * velocity * velocity;
    Vector3<> force{0, 0, lift};
    return {force, L_ELEVATOR.cross(force)};
}

Wrench<> AppliedLoads::getEnvironmentalLoads(const Vector<n> &state) {
    Quaternion<> quat{state[KF::q0], state[KF::q1], state[KF::q2], state[KF::q3]};
    Vector3<> weight = quat.rotate(WEIGHT);

    Vector3<> b_vel{state[KF::vx], state[KF::vy], state[KF::vz]}; // body velocity

    double drag = -DRAG_GAIN_BODY * b_vel.x * b_vel.x;
    double sin_of_angle_of_attack = b_vel.z / b_vel.x;
    double lift = -LIFT_GAIN_BODY * sin_of_angle_of_attack * (b_vel.x * b_vel.x + b_vel.z + b_vel.z);
    Vector3<> body_force{drag, 0, lift};

    double sin_of_rudder_angle_of_attack = b_vel.y / b_vel.x;
    double rudder_lift = -LIFT_GAIN_RUDDER * sin_of_rudder_angle_of_attack * (b_vel.x * b_vel.x + b_vel.y * b_vel.y);
    Vector3<> rudder_force{0, rudder_lift, 0};

    Vector3<> torque = L_BODY.cross(body_force) + L_RUDDER.cross(rudder_force);
    return {weight + body_force + rudder_force, torque};
}

Wrench<> AppliedLoads::getAppliedLoads(const Vector<n> &state) const {
    double velocity = state[KF::vx];
    return getPropellerLoads() +
           getRightAileronLoads(velocity) +
           getLeftAileronLoads(velocity) +
           getElevatorLoads(velocity) +
           getEnvironmentalLoads(state);
}

void AppliedLoads::updateWrapper(const double *control_inputs, const double *aircraft_state, double *forces,
                                 double *torques) {
    Vector<4> control_inputsVec{control_inputs};
    Vector<n> aircraft_state_vec{aircraft_state};

    update(ControlInputs::parseU(control_inputsVec));
    Wrench<> wrench = getAppliedLoads(aircraft_state_vec);
    for (size_t i = 0; i < 3; i++) {
        forces[i] = wrench.force[i];
        torques[i] = wrench.torque[i];
    }
}

Matrix<6, n> AppliedLoads::getAppliedLoadsJacobian(const Vector<n> &state) const {
    Matrix<6, n> wrench_jac = Matrix<6, n>::zeros();
    // propeller loads does not contribute since it does not depend on state

    // right aileron loads
    double vel_to_lift_jac = LIFT_GAIN_AILERON * utils::saturation<double>(current_control_inputs.right_aileron_angle, M_PI_4) * 2 * state[KF::vx];
    wrench_jac[2][KF::vx] -= vel_to_lift_jac;
    Vector3<> vel_to_right_torque_jac = L_RIGHT_AILERON.cross({0, 0, -vel_to_lift_jac});
    for (size_t i = 0; i < 3; i++) wrench_jac[3 + i][KF::vx] += vel_to_right_torque_jac[i];

    // left aileron loads
    vel_to_lift_jac = LIFT_GAIN_AILERON * utils::saturation<double>(current_control_inputs.left_aileron_angle, M_PI_4) * 2 * state[KF::vx];
    wrench_jac[2][KF::vx] -= vel_to_lift_jac;
    Vector3<> vel_to_left_torque_jac = L_LEFT_AILERON.cross({0, 0, -vel_to_lift_jac});
    for (size_t i = 0; i < 3; i++) wrench_jac[3 + i][KF::vx] += vel_to_left_torque_jac[i];

    // elevator loads
    vel_to_lift_jac = LIFT_GAIN_ELEVATOR * utils::saturation<double>(current_control_inputs.elevator_angle, M_PI_4) * 2 * state[KF::vx];
    wrench_jac[2][KF::vx] += vel_to_lift_jac;
    Vector3<> vel_to_elevator_torque_jac = L_ELEVATOR.cross({0, 0, vel_to_lift_jac});
    for (size_t i = 0; i < 3; i++) wrench_jac[3 + i][KF::vx] += vel_to_elevator_torque_jac[i];

    // TODO: environmental loads

    return wrench_jac;
}
