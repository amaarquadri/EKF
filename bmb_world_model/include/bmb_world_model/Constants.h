#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_utilities/ControllerGains.h>

// TODO: move to yaml file and read from parameter server

// environmental constants
static constexpr double GRAVITATIONAL_ACCELERATION = 9.81;  // m/s^2
static const Vector3<double> EARTH_GRAVITY{0, 0, GRAVITATIONAL_ACCELERATION};
static constexpr double ATMOSPHERIC_PRESSURE = 101325;  // Pa
static constexpr double AIR_DENSITY = 1.225;            // kg/m^3
static constexpr double AIR_MOLAR_MASS = 0.0289644;     // kg/mol
static constexpr double GAS_CONSTANT = 8.3144598;       // J/(K mol)

// rail track constants
static constexpr double RAIL_WIDTH = 1.4351;  // m
static const Vector3<double> NORTH{1, 0, 0};  // NOLINT(cert-err58-cpp)

// camera constants
static constexpr double CAMERA_GAIN = 1;
static constexpr int HALF_CAMERA_HORIZONTAL_PIXELS = 1920 / 2;
static constexpr int HALF_CAMERA_VERTICAL_PIXELS = 1080 / 2;
static constexpr double OPTICAL_FLOW_VELOCITY_GAIN = 1.3;

// IMU constants
static const Vector3<double> IMU_OFFSET{
    1, -0.1, -0.01};  // m, distance that the IMU is forward, right,
                      //  and below the center of gravity

// Accelerometer and Gyroscope noise constants, currently unused
// https://commons.erau.edu/cgi/viewcontent.cgi?article=1001&context=pr-honors-coe
static const Matrix<double, 3, 3> ANG_ACCELERATION_SCALING{
    0.95, 0.96, 0.97, -0, 96, 0.98, 0.99, -0.97, -0.99, 1};
static const Vector3<double> ANG_ACCELERATION_BIAS{0.001, 0.002, 0.003};
static const Matrix<double, 3, 3> ANG_VELOCITY_SCALING{
    0.95, 0.96, 0.97, -0, 96, 0.98, 0.99, -0.97, -0.99, 1};
static const Vector3<double> ANG_VELOCITY_BIAS{0.001, 0.002, 0.003};

// GPS constants
static const Vector<double, 2> STARTING_COORDINATES{
    43.47308029580669, -80.54007051556243};   // lat long coordinates of E5
static const double EARTH_RADIUS = 6.3781e6;  // m

// aircraft inertial constants
static constexpr double MASS = 3.615;  // aircraft mass, kg
static const Vector3<double> WEIGHT{
    0, 0, MASS* GRAVITATIONAL_ACCELERATION};  // NOLINT(cert-err58-cpp)
static const Matrix<double, 3, 3> INERTIA_TENSOR{
    1, 0, 0, 0, 1, 0, 0, 0, 1};  // NOLINT(cert-err58-cpp)
static const Matrix<double, 3, 3> INERTIA_TENSOR_INV =
    INERTIA_TENSOR.inv();  // NOLINT(cert-err58-cpp)

// aircraft body aerodynamic constants
static constexpr double MIN_RADIUS_CURVATURE =
    30;  // calculated to be 23.5 m, at 50 km/h

static constexpr double T_SAMPLE = 1E-3;  // sampling period

// Kalman filter constants
static constexpr size_t n = 25;  // number of states
static constexpr size_t p =
    bmb_msgs::SensorMeasurements::SIZE;  // number of sensor measurements

// controller constants
static const ControllerGains THROTTLE_GAIN{1, 0, 0, 1};
static const ControllerGains ROLL_GAIN{1, 0, 0};
static const ControllerGains PITCH_GAIN{1, 0, 0};
static const ControllerGains ELEVATOR_GAIN{1, 1, 1, 1};
static constexpr double PROPELLER_K_P = 1;
static const ControllerGains AILERON_GAIN{1, 1, 1, 1};
static const ControllerGains ALTITUDE_GAIN{1, 1, 1};
static constexpr double BASELINE_VELOCITY = 10;  // m/s
static constexpr double TRIM =
    7 * M_PI / 180.0;  // rad. This is used for sin of trim. Alternatively can
                       // use length ratios
static constexpr double SAMPLING_TIME = 1e-3;  // s
