#pragma once

#include <bmb_math/Accel.h>
#include <bmb_math/Matrix.h>
#include <bmb_math/Vector.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_world_model/Constants.h>
#include <utility>

bmb_msgs::SensorMeasurements getSensorMeasurements(
    const bmb_msgs::AircraftState& state,
    const Vector3<double>& accelerometer_bias,
    const Vector3<double>& gyroscope_bias, const Accel<double>& accel);

std::tuple<Matrix<double, bmb_msgs::SensorMeasurements::SIZE,
                  bmb_msgs::AircraftState::SIZE>,
           Matrix<double, bmb_msgs::SensorMeasurements::SIZE, 3>,
           Matrix<double, bmb_msgs::SensorMeasurements::SIZE, 3>,
           Matrix<double, bmb_msgs::SensorMeasurements::SIZE, 6>>
getSensorMeasurementsJacobian(const bmb_msgs::AircraftState& state,
                              const Vector3<double>& accelerometer_bias,
                              const Vector3<double>& gyroscope_bias,
                              const Accel<double>& accel);
