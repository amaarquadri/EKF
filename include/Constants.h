#pragma once

#include "Vector3.h"
#include "Matrix.h"

// environmental constants
static constexpr const double GRAVITATIONAL_ACCELERATION = 9.81; // m/s^2
static constexpr const double ATMOSPHERIC_PRESSURE = 101325; // Pa
static constexpr const double AIR_DENSITY = 1.225; // kg/m^3

// rail track constants
static constexpr const double RAIL_WIDTH = 1.4351; // m
static const Vector3<double> NORTH{1, 0, 0}; // NOLINT(cert-err58-cpp)

// camera constants
static constexpr const double CAMERA_GAIN = 1;
static constexpr const int HALF_CAMERA_HORIZONTAL_PIXELS = 1920 / 2;
static constexpr const int HALF_CAMERA_VERTICAL_PIXELS = 1080 / 2;
static constexpr const double OPTICAL_FLOW_VELOCITY_GAIN = 1.3;

// aircraft inertial constants
static constexpr const double MASS = 1; // aircraft mass, kg
static const Vector3<double> WEIGHT{0, 0, MASS * GRAVITATIONAL_ACCELERATION}; // NOLINT(cert-err58-cpp)
static const Matrix<double, 3, 3> INERTIA_TENSOR{1, 0, 0, 0, 1, 0, 0, 0, 1}; // NOLINT(cert-err58-cpp)
static const Matrix<double, 3, 3> INERTIA_TENSOR_INV = INERTIA_TENSOR.inv(); // NOLINT(cert-err58-cpp)

// aileron aerodynamic constant
static constexpr const double LIFT_GAIN_AILERON = 1;
// displacement of aerodynamic center of ailerons from the center of mass
static const Vector3<double> L_RIGHT_AILERON{0, 1, 0}; // NOLINT(cert-err58-cpp)
static const Vector3<double> L_LEFT_AILERON{L_RIGHT_AILERON[0], -L_RIGHT_AILERON[1], L_RIGHT_AILERON[2]}; // NOLINT(cert-err58-cpp)

// aircraft body aerodynamic constants
static constexpr const double DRAG_GAIN_BODY = 1;
static constexpr const double LIFT_GAIN_BODY = 1;
// displacement of aerodynamic center of the body from the center of mass
static const Vector3<double> L_BODY{0, 0, 0}; // NOLINT(cert-err58-cpp)

// rudder aerodynamic constant
static constexpr const double LIFT_GAIN_RUDDER = 1;
// displacement of aerodynamic center of rudder from the center of mass
static const Vector3<double> L_RUDDER{-1, 0, 0}; // NOLINT(cert-err58-cpp)

static constexpr const double T_SAMPLE = 1E-3; //sampling period

//propeller constants
static constexpr const double K_PROPELLER = 1;
static constexpr const double TAU_PROPELLER = 1;
static constexpr const double THRUST_GAIN_PROPELLER = 1;
static constexpr const double TORQUE_GAIN_PROPELLER = 1;
static const Vector3<double> L_FRONT_PROPELLER{0, 0, 0}; // NOLINT(cert-err58-cpp)

//elevator constants
static constexpr const double LIFT_GAIN_ELEVATOR = 1;
static const Vector3<double> L_ELEVATOR{0, 0, 0}; // NOLINT(cert-err58-cpp)

// Kalman filter constants
constexpr static const int n = 25; // number of states
constexpr static const int p = 18; // number of sensor measurements
