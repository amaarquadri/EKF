#pragma once

#include "KF.h"

class UKF : public KF {
private:
    Matrix<2 * n + 1, n> sigma = Matrix<2 * n + 1, n>::zeros();
    Matrix<n, n> P_cholesky = Matrix<n, n>::identity();
    constexpr static const double alpha = 0.1; // scaling factor for the spread of
    // the sigma points around the mean 0 <= alpha <= 1 usually small
    constexpr static const double kappa = 0; // secondary scaling factor
    constexpr static const double beta = 2; // parameter to affect the weighting of the zeroth sigma point for the calculation of the covariance
    constexpr static const double lambda = alpha * alpha * (n + kappa) - n; // scaling factor for sigma points
    static const double gamma; //multiply by covariance for sigma points
    Vector<2 * n + 1> state_weights;
    Vector<2 * n + 1> covariance_weights;

    void updateKF(const SensorMeasurements<> &sensorMeasurements,
                  const double &dt) override;

public:
    UKF();
};

