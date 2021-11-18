#pragma once

#include "KF.h"
#include "Expression.h"
#include "Matrix.h"
#include "Matrix3D.h"
#include <utility>

class EKF : public KF {
public:
    void updateKF(const SensorMeasurements<> &sensorMeasurements,
                  const double &dt) override;

private:
    static const Matrix3D<4, 4, 3, ExprPtr> QUAT_TO_QUAT_JAC_EXPR;
    static const Matrix<6, 6> WRENCH_TO_ACCEL_JAC;

    static std::pair<Matrix<n, n>, Matrix<n, 6>> fJacobian(const Vector<n> &x, const double &dt);
};
