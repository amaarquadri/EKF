#pragma once

#include "SensorFilter.h"
#include "Vector.h"
#include "Matrix.h"
#include "Vector3.h"
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
    Vector<double, n> x{};
    Matrix<double, n, n> P = Matrix<double, n, n>::identity();
    const Matrix<double, n, n> Q = Matrix<double, n, n>::identity();
    const Matrix<double, p, p> R = Matrix<double, p, p>::identity();
    AppliedLoads applied_loads{};
    Wrench<double> current_loads; // stores applied_loads.getAppliedLoads(x) for the current time-step

public:
    KF();

    void update(const SensorMeasurements &sensorMeasurements,
                const ControlInputs &control_inputs, double dt) final;

    virtual void updateKF(const SensorMeasurements &sensorMeasurements, const double &dt) = 0;

    [[nodiscard]] AircraftState getOutput() const final;

protected:
    [[nodiscard]] Vector<double, n> f(const Vector<double, n> &state, const double &dt) const;

    static Vector<double, p> h(const Vector<double, n> &state, const double &dt);
};


