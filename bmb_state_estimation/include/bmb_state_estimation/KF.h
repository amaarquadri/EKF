#pragma once

#include <bmb_math/Accel.h>
#include <bmb_math/Matrix.h>
#include <bmb_math/Vector.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_state_estimation/SensorFilter.h>
#include <bmb_world_model/Constants.h>
#include <cstddef>

class KF : public SensorFilter {
 public:
  enum : size_t {
    px,
    py,
    pz,  // position in earth frame
    q0,
    q1,
    q2,
    q3,  // rotation from earth frame to body frame
    vx,
    vy,
    vz,  // velocity in body frame
    wx,
    wy,
    wz,  // angular velocity in body frame
    magx,
    magy,
    magz,  // magnetic field vector in body frame
    accel_bx,
    accel_by,
    accel_bz,  // accelerometer bias in body frame
    gyro_bx,
    gyro_by,
    gyro_bz,  // gyroscope bias in body frame
    mag_bx,
    mag_by,
    mag_bz  // magnetometer bias in body frame
  };

 protected:
  Vector<double, n> x{};
  Matrix<double, n, n> P = Matrix<double, n, n>::identity();
  const Matrix<double, n, n> Q = Matrix<double, n, n>::identity();
  const Matrix<double, p, p> R = Matrix<double, p, p>::identity();
  Wrench<double>
      current_loads;  // stores the applied loads for the current time-step
  Accel<double> current_accel;  // stores accelerations and angular
                                // accelerations based on current_loads

 public:
  KF();

  void update(const bmb_msgs::SensorMeasurements& sensor_measurements,
              const bmb_msgs::ControlInputs& control_inputs,
              const double& dt) final;

  virtual void updateKF(const bmb_msgs::SensorMeasurements& sensor_measurements,
                        const bmb_msgs::ControlInputs& control_inputs,
                        const double& dt) = 0;

  [[nodiscard]] bmb_msgs::AircraftState getOutput() const final;

 protected:
  [[nodiscard]] Vector<double, n> f(const Vector<double, n>& state,
                                    const double& dt) const;

  [[nodiscard]] Vector<double, p> h(const Vector<double, n>& state,
                                    const double& dt);
};
