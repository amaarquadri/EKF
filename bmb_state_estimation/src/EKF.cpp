#include "bmb_state_estimation/EKF.h"
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_math/Matrix.h>
#include <bmb_math/Matrix3D.h>
#include <bmb_math/MessageUtilities.h>
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_world_model/AppliedLoads.h>
#include <bmb_world_model/Constants.h>
#include <bmb_world_model/SensorModels.h>
#include <string>
#include <unordered_map>

static Matrix3D<ExprPtr, 4, 4, 3> getQuatToQuatJacExpr() {
  Matrix3D<ExprPtr, 4, 4, 3> expr;

  const Quaternion<ExprPtr> quat{Variable::make("q0"), Variable::make("q1"),
                                 Variable::make("q2"), Variable::make("q3")};
  const Matrix<ExprPtr, 4, 3> mat = quat.E().transpose() * quat.cong().toDCM();
  for (size_t i = 0; i < 4; i++)
    for (size_t j = 0; j < 4; j++)
      for (size_t k = 0; k < 3; k++)
        expr[i][j][k] = mat[j][k]
                            ->diff(std::static_pointer_cast<Variable>(quat[i])
                                       ->getIdentifier())
                            ->simplify();

  return expr;
}

static Matrix<double, 6, 6> getWrenchToAccelJac() {
  Matrix<double, 6, 6> mat = Matrix<double, 6, 6>::zeros();
  for (size_t i = 0; i < 3; i++) {
    mat[i][i] = 1 / MASS;
    for (size_t j = 0; j < 3; j++) mat[i][j] = INERTIA_TENSOR_INV[i][j];
  }

  return mat;
}

const Matrix3D<ExprPtr, 4, 4, 3> EKF::QUAT_TO_QUAT_JAC_EXPR =
    getQuatToQuatJacExpr();  // NOLINT(cert-err58-cpp)
const Matrix<double, 6, 6> EKF::WRENCH_TO_ACCEL_JAC =
    getWrenchToAccelJac();  // NOLINT(cert-err58-cpp)

void EKF::updateKF(const bmb_msgs::SensorMeasurements& sensor_measurements,
                   const bmb_msgs::ControlInputs& control_inputs,
                   const double& dt) {
  // Jacobian naming convention:
  // a_to_b_jac represents the derivative of b with respect to a, and is a
  // matrix of size (b, a). The ith row and jth column of a_to_b_jac represents
  // the derivative of the ith element of b with respect to the jth element of a
  // In other words, how does a impact b
  // For simplicity, x_to_a_jac is simply written as a_jac when x represents the
  // state. By the chain rule, b_to_c_jac * a_to_b_jac = a_to_c_jac
  const bmb_msgs::AircraftState state = getOutput();
  const Vector3 accelerometer_bias = x.slice<accel_bx, accel_bz + 1>();
  const Vector3 gyro_bias = x.slice<gyro_bx, gyro_bz + 1>();
  const Matrix<double, 6, bmb_msgs::AircraftState::SIZE> wrench_jac =
      getAppliedLoadsJacobian(
          state,
          control_inputs);  // derivative of wrench with respect to state
  const Matrix<double, 6, bmb_msgs::AircraftState::SIZE> accel_jac =
      WRENCH_TO_ACCEL_JAC * wrench_jac;

  // prediction step
  auto [f_jac, accel_to_f_jac] = fJacobian(x, dt);
  f_jac +=
      accel_to_f_jac * accel_jac;  // relies on the fact that AircraftState's
                                   // elements are the first in KF's state

  x = f(x, dt);
  P = f_jac * P * f_jac.transpose() + Q;

  // update step
  const Vector<double, p> z = bmb_math::as_vector(sensor_measurements);
  const Vector<double, p> h_vec = h(x, dt);
  Matrix<double, p, n> h_jac = Matrix<double, p, n>::zeros();
  auto [aircraft_state_to_h_jac, accelerometer_bias_to_h_jac,
        gyro_bias_to_h_jac, accel_to_h_jac] =
      getSensorMeasurementsJacobian(state, accelerometer_bias, gyro_bias,
                                    current_accel);
  h_jac += aircraft_state_to_h_jac;
  h_jac.operator+=<0, accel_bx>(accelerometer_bias_to_h_jac);
  h_jac.operator+=<0, gyro_bx>(gyro_bias_to_h_jac);
  h_jac += accel_to_h_jac * accel_jac;
  const Matrix<double, n, p> h_jac_transpose = h_jac.transpose();

  const Vector<double, p> y = z - h_vec;
  // multiplication order doesn't matter, both require n*p*(n+p) multiplications
  // regardless of order
  const Matrix<double, p, p> S = h_jac * P * h_jac_transpose + R;
  const Matrix<double, n, p> K = P * h_jac_transpose * S.inv();

  x += K * y;
  P -= K * h_jac * P;
}

std::pair<Matrix<double, n, n>, Matrix<double, n, 6>> EKF::fJacobian(
    const Vector<double, n>& x, const double& dt) {
  Matrix<double, n, n> f_jac =
      Matrix<double, n,
             n>::zeros();  // derivative of state with respect to state
  Matrix<double, n, 6> accel_jac =
      Matrix<double, n,
             6>::zeros();  // derivative of state with respect to accel

  // setup some basic variables for use later
  const Quaternion<double> quat{x[q0], x[q1], x[q2], x[q3]};
  const Matrix<double, 3, 3> DCM_inv = quat.cong().toDCM();
  const Vector3<double> w_abs =
      quat.unrotate(Vector3<double>{x[wx], x[wy], x[wz]});
  const Quaternion<double> quat_new =
      quat + quat.E().transpose() * w_abs * (dt / 2);
  const std::unordered_map<std::string, double> subs = {
      {"q0", x[q0]}, {"q1", x[q1]}, {"q2", x[q2]}, {"q3", x[q3]}};

  const Matrix3D<double, 4, 4, 3> mat = QUAT_TO_QUAT_JAC_EXPR.applyFunc<double>(
      [&subs](const ExprPtr& expr) { return expr->evaluate(subs); });
  const Matrix<double, 4, 4> quat_to_quat_jac =
      mat * Vector3<double>{x[wx], x[wy], x[wz]} * (dt / 2);

  const Matrix<double, 4, 3> w_to_quat_jac =
      quat.E().transpose() * DCM_inv * dt / 2;

  // TODO: magnetic field derivatives with respect to quaternion and angular
  // velocity
  /**
   * Derivation:
   * mag_new = quat_new.toDCM() * quat.cong().toDCM() * (mag - mag_b) + mag_b
   * Define M = quat_new.toDCM() * quat.cong().toDCM()
   * mag_new = M * (mag - mag_b) + mag_b
   * (mag_new - mag) / dt = (M * (mag - mag_b) + mag_b - mag) / dt
   * (mag_new - mag) / dt = (M - I) / dt * (mag - mag_b)
   * Thus, mag_to_mag_jac = (M - I) / dt and mag_mag_b_jac = -mag_to_mag_jac
   */
  const Matrix<double, 3, 3> mag_to_mag_jac =
      (quat_new.toDCM() * DCM_inv - Matrix<double, 3, 3>::identity()) / dt;

  // fill in the separate elements into f_jac
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      f_jac[px + i][vx + j] =
          DCM_inv[i][j] *
          dt;  // derivative of position with respect to velocity
      f_jac[magx + i][magx + j] =
          mag_to_mag_jac[i][j];  // derivative of magnetic field with respect to
                                 // itself
      f_jac[magx + i][mag_bx + j] =
          -mag_to_mag_jac[i][j];  // derivative of magnetic field with respect
                                  // to magnetic field bias
      accel_jac[wx + i][3 + j] = dt;  // derivative of angular velocity with
                                      // respect to angular accelerations
    }
    accel_jac[vx + i][i] =
        dt;  // derivative of velocity with respect to accelerations
  }
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++)
      f_jac[q0 + i][q0 + j] =
          quat_to_quat_jac[i][j];  // derivative of quaternion with respect to
                                   // itself
    for (size_t j = 0; j < 3; j++)
      f_jac[q0 + i][wx + j] =
          w_to_quat_jac[i][j];  // derivative of quaternion with respect to
                                // angular velocity
  }

  return {f_jac, accel_jac};
}
