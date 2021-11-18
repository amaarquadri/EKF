#pragma once

#include "SensorFilter.h"
#include "Vector.h"
#include "Matrix.h"
#include "Vector3.h"
#include "Wrench.h"
#include "Accel.h"
#include "AppliedLoads.h"
#include "Constants.h"

class KF : public SensorFilter {
public:
    enum : size_t {
        px, py, pz,  // position in earth frame
        q0, q1, q2, q3,  // rotation from earth frame to body frame
        vx, vy, vz,  // velocity in body frame
        wx, wy, wz,  // angular velocity in body frame
        magx, magy, magz,  // magnetic field vector in body frame
        accel_bx, accel_by, accel_bz,  // accelerometer bias in body frame
        gyro_bx, gyro_by, gyro_bz,  // gyroscope bias in body frame
        mag_bx, mag_by, mag_bz  // magnetometer bias in body frame
    };

protected:
    Vector<n> x{};
    Matrix<n, n> P = Matrix<n, n>::identity();
    const Matrix<n, n> Q = Matrix<n, n>::identity();
    const Matrix<p, p> R = Matrix<p, p>::identity();
    AppliedLoads applied_loads{};
    Wrench<> current_loads; // stores applied_loads.getAppliedLoads(x) for the current time-step
    Accel<> current_accel; // stores accelerations and angular accelerations based on current_loads

public:
    KF();

    void update(const SensorMeasurements<> &sensorMeasurements,
                const ControlInputs &control_inputs, const double &dt) final;

    virtual void updateKF(const SensorMeasurements<> &sensorMeasurements, const double &dt) = 0;

    [[nodiscard]] AircraftState getOutput() const final;

protected:
    [[nodiscard]] Vector<n> f(const Vector<n> &state, const double &dt) const;

    [[nodiscard]] Vector<p> h(const Vector<n> &state, const double &dt);
};
