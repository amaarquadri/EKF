#include "bmb_state_estimation/UKF.h"
#include <bmb_math/Matrix.h>
#include <bmb_math/MessageUtilities.h>
#include <bmb_math/Vector.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <cmath>

const double UKF::gamma = std::sqrt(n + UKF::lambda);  // NOLINT(cert-err58-cpp)

UKF::UKF() {
  P_cholesky = P.cholesky();

  // state weights i = 0: w = lambda / (N + lambda) i = 1 to 2N: w = 1 / (2 * (N
  // + lambda))
  state_weights[0] = lambda / (n + lambda);
  for (size_t i = 1; i < 2 * n + 1; i++)
    state_weights[i] = 1 / (2 * (n + lambda));

  // covariance weights i = 0: w = lambda / (N + lambda) + (1 - alpha*alpha +
  // beta) i = 1 to 2N: w is same
  covariance_weights[0] = lambda / (n + lambda) + (1 - alpha * alpha + beta);
  for (size_t i = 1; i < 2 * n + 1; i++)
    covariance_weights[i] = 1 / (2 * (n + lambda));
}

void UKF::updateKF(const bmb_msgs::SensorMeasurements& sensor_measurements,
                   const bmb_msgs::ControlInputs& /** control_inputs **/,
                   const double& dt) {
  Vector<double, n> x_new = f(x, dt);
  auto new_sigma = sigma;

  for (size_t j = 0; j < n; j++) {  // i is column number, j is row number
    new_sigma[0][j] = this->x[j];
    for (size_t i = 1; i <= n; i++)
      new_sigma[i][j] = this->x[i] + P_cholesky[i][j] * gamma;
    for (size_t i = n + 1; i <= 2 * n; i++)
      new_sigma[i][j] = this->x[i] - P_cholesky[i][j] * gamma;
  }

  // transform sigma through state update function. this is state sigma
  auto state_sigma = new_sigma;
  for (size_t i = 0; i < 2 * n + 1; i++) state_sigma[i] = f(new_sigma[i], dt);

  // for i=0 to 2n state estimate += state weights[i] times state sigma[i]
  Vector<double, n> state_estimate;
  for (size_t i = 0; i < 2 * n + 1; i++)
    state_estimate += state_sigma[i] * state_weights[i];

  // for i=0 to 2n state covariance += covariance weight[i] times state sigma[i]
  // - state estimate dot itself
  Matrix<double, n, n> state_covariance;
  for (size_t i = 0; i < 2 * n + 1; i++)
    state_covariance +=
        Matrix<double, n, n>::outerProduct((state_sigma[i] - state_estimate),
                                           (state_sigma[i] - state_estimate)) *
        covariance_weights[i];

  // transform state sigma through measurement function
  Matrix<double, 2 * n + 1, p> measurement_sigma;
  for (size_t i = 0; i < 2 * n + 1; i++)
    measurement_sigma[i] = h(state_sigma[i], dt);

  // mean measurement vector += state weights[i] times measurement sigma[i]
  Vector<double, p> measurement_estimate;
  for (size_t i = 0; i < 2 * n + 1; i++)
    measurement_estimate += measurement_sigma[i] * state_weights[i];

  // measurement covariance += covariance weights times transformed sigma points
  // minus mean measurement vector dot itself
  Matrix<double, p, p> measurement_covariance;
  for (size_t i = 0; i < 2 * n + 1; i++)
    measurement_covariance +=
        Matrix<double, p, p>::outerProduct(
            (measurement_sigma[i] - measurement_estimate),
            (measurement_sigma[i] - measurement_estimate)) *
        covariance_weights[i];

  // cross covariance is state covariance and measurement covariance
  Matrix<double, n, p> cross_covariance;
  for (size_t i = 0; i < 2 * n + 1; i++)
    cross_covariance += Matrix<double, n, p>::outerProduct(
                            (state_sigma[i] - state_estimate),
                            (measurement_sigma[i] - measurement_estimate)) *
                        covariance_weights[i];

  // Kalman gain is cross covariance times inverse of measurement covariance
  Matrix<double, n, p> K;
  K = cross_covariance * measurement_covariance.inv();

  // kalman gain update
  x = state_estimate +
      K * (bmb_math::as_vector(sensor_measurements) - measurement_estimate);
  P = state_covariance - K * measurement_covariance * K.transpose();
  // update P_cholesky
  P_cholesky = P.cholesky();
}

// TODO: More efficient update using
//  https://www.researchgate.net/publication/220398698_Two_Efficient_Implementation_Forms_of_Unscented_Kalman_Filter
//  which could be 40% faster
