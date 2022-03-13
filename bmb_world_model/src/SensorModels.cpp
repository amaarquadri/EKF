#include "bmb_world_model/SensorModels.h"
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_world_model/Constants.h>
#include <cmath>
#include <utility>

// determines whether noise is added to the sensor measurements
#define SENSOR_NOISE 1

#if SENSOR_NOISE == 1
#include <chrono>
#include <random>
static unsigned seed = std::chrono::system_clock::now()
                           .time_since_epoch()
                           .count();  // NOLINT(cert-err58-cpp)
static auto random_engine =
    std::default_random_engine(seed);  // NOLINT(cert-err58-cpp)
static std::normal_distribution<double> optical_flow_noise(
    0, 1);  // NOLINT(cert-err58-cpp)
#endif

static void railDetection(const Quaternion<double>& quat,
                          const double& altitude,
                          bmb_msgs::SensorMeasurements& sensor_measurements) {
  if (altitude < 0) {
    // cannot see rail if we are underground
    // leave found_rail as false and leave rail_pixel_x, rail_pixel_y,
    // rail_pixel_width, and rail_angle as 0
    return;
  }

  const Vector3<double> down_earth =
      quat.unrotate({0, 0, 1});  // down (i.e. the direction the camera is
                                 // pointing) in earth coordinates
  if (down_earth.z < 0) {
    // cannot see rail if the plane is upside down
    return;
  }

  // rail offset in body coordinates
  const Vector3<double> rail_offset_body =
      quat.rotate(down_earth * altitude / down_earth.z);

  const Vector3<double> rail_pixel_location =
      rail_offset_body * CAMERA_GAIN / rail_offset_body.z;
  if (HALF_CAMERA_HORIZONTAL_PIXELS < std::abs(rail_pixel_location.x) ||
      HALF_CAMERA_VERTICAL_PIXELS < std::abs(rail_pixel_location.y)) {
    // rail tracks are out of the field of view of the camera
    return;
  }

  // rail tracks are visible
  sensor_measurements.rail_detection.found = true;
  sensor_measurements.rail_detection.pixel_x = rail_pixel_location.x;
  sensor_measurements.rail_detection.pixel_y = rail_pixel_location.y;
  sensor_measurements.rail_detection.pixel_width =
      RAIL_WIDTH * CAMERA_GAIN / rail_offset_body.z;

  const Vector3<double> rail_direction_body =
      quat.rotate(NORTH);  // rail direction in body coordinates
  const Vector3<double> rail_direction_pixels =
      rail_direction_body * CAMERA_GAIN / rail_offset_body.z;
  sensor_measurements.rail_detection.angle =
      std::atan2(rail_direction_pixels.y, rail_direction_pixels.x);
}

static void opticalFlow(const Vector3<double>& velocity, const double& altitude,
                        bmb_msgs::SensorMeasurements& sensor_measurements) {
  const Vector3<double> pixel_velocity =
      velocity * OPTICAL_FLOW_VELOCITY_GAIN / altitude;
  sensor_measurements.optical_flow_reading.x_pixel_velocity = pixel_velocity.x;
  sensor_measurements.optical_flow_reading.y_pixel_velocity = pixel_velocity.y;
#if SENSOR_NOISE == 1
  sensor_measurements.optical_flow_reading.x_pixel_velocity +=
      optical_flow_noise(random_engine);
  sensor_measurements.optical_flow_reading.y_pixel_velocity +=
      optical_flow_noise(random_engine);
#endif
}

static void altitudeSensor(const double& altitude,
                           bmb_msgs::SensorMeasurements& sensor_measurements) {
  // temperature is a function of height, location and time of year so this
  // might change
  static constexpr double temperature = 288.15;  // K
  sensor_measurements.pressure_reading.fluid_pressure =
      ATMOSPHERIC_PRESSURE * exp(-GRAVITATIONAL_ACCELERATION * AIR_MOLAR_MASS *
                                 altitude / (GAS_CONSTANT * temperature));
}

static void imu(const Quaternion<double>& quat,
                const Vector3<double>& body_acceleration,
                const Vector3<double>& body_ang_acceleration,
                const Vector3<double>& body_ang_velocity,
                const Vector3<double>& accelerometer_bias,
                const Vector3<double>& gyroscope_bias,
                bmb_msgs::SensorMeasurements& sensor_measurements) {
  const Vector3<double> body_gravity = quat.rotate(EARTH_GRAVITY);
  const Vector3<double> measured_acceleration =
      body_acceleration +
      body_ang_velocity.cross(body_ang_velocity.cross(IMU_OFFSET)) +
      body_ang_acceleration.cross(IMU_OFFSET) -
      body_gravity;  // idealize measured acceleration
  // TODO: implement noise stuff from here:
  // https://www.mathworks.com/help/aeroblks/threeaxisaccelerometer.html
  //  and here: https://www.mathworks.com/help/aeroblks/threeaxisgyroscope.html
  Vector3<double>{measured_acceleration + accelerometer_bias}.copy_to(
      sensor_measurements.imu_reading.linear_acceleration);
  Vector3<double>{body_ang_velocity + gyroscope_bias}.copy_to(
      sensor_measurements.imu_reading.angular_velocity);
}

static void gps(const Vector3<double>& position,
                bmb_msgs::SensorMeasurements& sensor_measurements) {
  // Based on: https://www.movable-type.co.uk/scripts/latlong.html
  const double rect_dist = std::hypot(position.x, position.y);
  sensor_measurements.gps_reading.latitude =
      asin(sin(STARTING_COORDINATES[0]) * cos(rect_dist / EARTH_RADIUS) +
           cos(STARTING_COORDINATES[0]) * sin(rect_dist / EARTH_RADIUS) *
               position[1] / rect_dist);
  sensor_measurements.gps_reading.longitude =
      STARTING_COORDINATES[1] +
      atan2(position[0] / rect_dist * sin(rect_dist / EARTH_RADIUS) *
                cos(STARTING_COORDINATES[0]),
            cos(rect_dist / EARTH_RADIUS) -
                sin(STARTING_COORDINATES[0]) *
                    sin(sensor_measurements.gps_reading.latitude));
  sensor_measurements.gps_reading.altitude = -position.z;
}

bmb_msgs::SensorMeasurements getSensorMeasurements(
    const bmb_msgs::AircraftState& state,
    const Vector3<double>& accelerometer_bias,
    const Vector3<double>& gyroscope_bias, const Accel<double>& accel) {
  const Vector3<double> position{state.pose.position};
  const Quaternion<double> quat{state.pose.orientation};
  const Vector3<double> velocity{state.twist.linear};
  const double altitude = -position.z;

  bmb_msgs::SensorMeasurements sensor_measurements;
  altitudeSensor(altitude, sensor_measurements);
  gps(position, sensor_measurements);
  imu(quat, accel.linear, accel.angular, Vector3<double>{state.twist.angular},
      accelerometer_bias, gyroscope_bias, sensor_measurements);
  railDetection(quat, altitude, sensor_measurements);
  opticalFlow(velocity, altitude, sensor_measurements);

  return sensor_measurements;
}

std::tuple<Matrix<double, p, bmb_msgs::AircraftState::SIZE>,
           Matrix<double, p, 3>, Matrix<double, p, 3>, Matrix<double, p, 6>>
getSensorMeasurementsJacobian(const bmb_msgs::AircraftState& state,
                              const Vector3<double>& accelerometer_bias,
                              const Vector3<double>& gyroscope_bias,
                              const Accel<double>& accel) {
  using namespace bmb_msgs;
  auto h_jac = Matrix<double, p, bmb_msgs::AircraftState::SIZE>::zeros();
  auto accelerometer_bias_to_h_jac = Matrix<double, p, 3>::zeros();
  auto gyroscope_bias_to_h_jac = Matrix<double, p, 3>::zeros();
  auto accel_to_h_jac = Matrix<double, p, 6>::zeros();

  // TODO
  h_jac[SensorMeasurements::PRESSURE][AircraftState::PX] =
      AIR_DENSITY * GRAVITATIONAL_ACCELERATION;

  h_jac[SensorMeasurements::IMU_wx][AircraftState::WX] = 1;
  h_jac[SensorMeasurements::IMU_wy][AircraftState::WY] = 1;
  h_jac[SensorMeasurements::IMU_wz][AircraftState::WZ] = 1;

  h_jac[SensorMeasurements::ALT][AircraftState::PZ] = -1;

  return {h_jac, accelerometer_bias_to_h_jac, gyroscope_bias_to_h_jac,
          accel_to_h_jac};
}
