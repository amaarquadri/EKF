#pragma once

#include "Wrench.h"
#include "Vector.h"
#include "Matrix.h"
#include "ControlInputs.h"
#include "Constants.h"
#include "Expression.h"
#include "TransferFunction.h"

class AppliedLoads {
public:
    void update(const ControlInputs &control_inputs);

    void updateWrapper(const double *control_inputs, const double *aircraft_state, double *forces, double *torques);

    [[nodiscard]] Wrench<double> getAppliedLoads(const Vector<double, n> &state) const;

    [[nodiscard]] Matrix<double, 6, n> getAppliedLoadsJacobian(const Vector<double, n> &state) const;

private:
    static const Matrix<ExprPtr, 3, 4> QUAT_TO_WEIGHT_JAC_EXPR;

    ControlInputs current_control_inputs{0, 0, 0, 0};
    const TransferFunction<double, 2, 2> ang_vel_volt_tf = c2d(TransferFunction<double, 1, 2>{K_PROPELLER, 1, TAU_PROPELLER},
                                                               LOW_LEVEL_SAMPLE_TIME);

    [[nodiscard]] double getPropellerAngVelocity() const;

    [[nodiscard]] Wrench<double> getPropellerLoads() const;

    [[nodiscard]] Wrench<double> getRightAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getLeftAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getElevatorLoads(const double &velocity) const;

    static Wrench<double> getEnvironmentalLoads(const Vector<double, n> &state);
};
