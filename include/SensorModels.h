#pragma once

#include "Vector.h"
#include "Accel.h"
#include "Matrix.h"
#include "SensorMeasurements.h"
#include "Constants.h"
#include <utility>

SensorMeasurements<> getSensorMeasurements(const Vector<n> &state, const Accel<> &accel);

void getSensorMeasurementsWrapper(const double *aircraft_state, double *double_sensor_measurements,
                                  uint8_t *uint8_sensor_measurements, unsigned char *bool_sensor_measurements);

std::pair<Matrix<p, n>, Matrix<p, 6>> getSensorMeasurementsJacobian(const Vector<n> &state, const Accel<> &current_loads);
