#pragma once

#include "Wrench.h"
#include "Vector.h"
#include "Matrix.h"
#include "ControlInputs.h"
#include "Constants.h"
#include "Expression.h"

class AppliedLoads {
public:
    void update(const ControlInputs &control_inputs);

    void updateWrapper(const double *control_inputs, const double *aircraft_state, double *forces, double *torques);

    [[nodiscard]] Wrench<> getAppliedLoads(const Vector<n> &state) const;

    [[nodiscard]] Matrix<6, n> getAppliedLoadsJacobian(const Vector<n> &state) const;

private:
    static const Matrix<3, 4, ExprPtr> QUAT_TO_WEIGHT_JAC_EXPR;

    ControlInputs current_control_inputs{0, 0, 0, 0};
    ControlInputs last_control_inputs{0, 0, 0, 0};
    double last_propeller_ang_vel = 0;

    [[nodiscard]] double getPropellerAngVelocity() const;

    [[nodiscard]] Wrench<> getPropellerLoads() const;

    [[nodiscard]] Wrench<> getRightAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<> getLeftAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<> getElevatorLoads(const double &velocity) const;

    static Wrench<> getEnvironmentalLoads(const Vector<n> &state);
};
