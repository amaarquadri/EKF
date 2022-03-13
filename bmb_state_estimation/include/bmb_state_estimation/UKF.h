#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Vector.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_state_estimation/KF.h>

class UKF : public KF {
 private:
  Matrix<double, 2 * n + 1, n> sigma = Matrix<double, 2 * n + 1, n>::zeros();
  Matrix<double, n, n> P_cholesky = Matrix<double, n, n>::identity();
  static constexpr double alpha = 0.1;  // scaling factor for the spread of
  // the sigma points around the mean 0 <= alpha <= 1 usually small
  static constexpr double kappa = 0;  // secondary scaling factor
  static constexpr double beta =
      2;  // parameter to affect the weighting of the zeroth sigma point for the
          // calculation of the covariance
  static constexpr double lambda =
      alpha * alpha * (n + kappa) - n;  // scaling factor for sigma points
  static const double gamma;  // multiply by covariance for sigma points
  Vector<double, 2 * n + 1> state_weights;
  Vector<double, 2 * n + 1> covariance_weights;

  void updateKF(const bmb_msgs::SensorMeasurements& sensor_measurements,
                const bmb_msgs::ControlInputs& control_inputs,
                const double& dt) override;

 public:
  UKF();
};
