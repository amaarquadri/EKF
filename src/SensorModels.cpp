#include "SensorModels.h"
#include "AppliedLoads.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "KF.h"
#include <cmath>

// determines whether noise is added to the sensor measurements
#define SENSOR_NOISE 1

#if SENSOR_NOISE == 1
#include <chrono>
#include <random>
static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); // NOLINT(cert-err58-cpp)
static auto random_engine = std::default_random_engine(seed); // NOLINT(cert-err58-cpp)
static std::normal_distribution<> optical_flow_noise(0, 1); // NOLINT(cert-err58-cpp)
#endif

static void railDetection(const Quaternion<> &quat, const double &altitude, SensorMeasurements<> &sensor_measurements) {
    if (altitude < 0) {
        // cannot see rail if we are underground
        // leave found_rail as false and leave rail_pixel_x, rail_pixel_y, rail_pixel_width, and rail_angle as 0
        return;
    }

    Vector3<> down_earth = quat.unrotate({0, 0, 1}); // down (i.e. the direction the camera is pointing) in earth coordinates
    if (down_earth.z < 0) {
        // cannot see rail if the plane is upside down
        return;
    }

    // rail offset in body coordinates
    Vector3<> rail_offset_body = quat.rotate(down_earth * altitude / down_earth.z);

    Vector3<> rail_pixel_location = rail_offset_body * CAMERA_GAIN / rail_offset_body.z;
    if (HALF_CAMERA_HORIZONTAL_PIXELS < std::abs(rail_pixel_location.x) ||
        HALF_CAMERA_VERTICAL_PIXELS < std::abs(rail_pixel_location.y)) {
        // rail tracks are out of the field of view of the camera
        return;
    }

    // rail tracks are visible
    sensor_measurements.found_rail = true;
    sensor_measurements.rail_pixel_x = rail_pixel_location.x;
    sensor_measurements.rail_pixel_y = rail_pixel_location.y;
    sensor_measurements.rail_pixel_width = RAIL_WIDTH * CAMERA_GAIN / rail_offset_body.z;

    Vector3<> rail_direction_body = quat.rotate(NORTH); // rail direction in body coordinates
    Vector3<> rail_direction_pixels = rail_direction_body * CAMERA_GAIN / rail_offset_body.z;
    sensor_measurements.rail_angle = std::atan2(rail_direction_pixels.y, rail_direction_pixels.x);
}

static void opticalFlow(const Vector3<> &velocity, const double &altitude, SensorMeasurements<> &sensor_measurements) {
    Vector3<> pixel_velocity = velocity * OPTICAL_FLOW_VELOCITY_GAIN / altitude;
    sensor_measurements.pixel_velocity[0] = pixel_velocity.x;
    sensor_measurements.pixel_velocity[1] = pixel_velocity.y;
#if SENSOR_NOISE == 1
    sensor_measurements.pixel_velocity[0] += optical_flow_noise(random_engine);
    sensor_measurements.pixel_velocity[1] += optical_flow_noise(random_engine);
#endif
}

static void altitudeSensor(const double &altitude, SensorMeasurements<> &sensor_measurements) {
    // temperature is a function of height and also location and also time of year so this might change
    double temperature = 288.15; // K
    sensor_measurements.pressure = ATMOSPHERIC_PRESSURE *
            exp(-GRAVITATIONAL_ACCELERATION * AIR_MOLAR_MASS * altitude / (GAS_CONSTANT * temperature));
}

static void imu(const Quaternion<> &quat,
                const Vector3<> &body_acceleration,
                const Vector3<> &body_ang_acceleration,
                const Vector3<> &body_ang_velocity,
                const Vector3<> &accelerometer_bias,
                const Vector3<> &gyroscope_bias,
                SensorMeasurements<> &sensor_measurements) {
    Vector3<> body_gravity = quat.rotate(EARTH_GRAVITY);
    Vector3<> measured_acceleration = body_acceleration +
                                            body_ang_velocity.cross(body_ang_velocity.cross(IMU_OFFSET)) +
                                            body_ang_acceleration.cross(IMU_OFFSET) -
                                            body_gravity; // idealize measured acceleration
    // TODO: implement noise stuff from here: https://www.mathworks.com/help/aeroblks/threeaxisaccelerometer.html
    //  and here: https://www.mathworks.com/help/aeroblks/threeaxisgyroscope.html
    sensor_measurements.imu_acceleration = measured_acceleration + accelerometer_bias;
    sensor_measurements.imu_angular_velocity = body_ang_velocity + gyroscope_bias;
}

static void gps(const Vector3<> &position, SensorMeasurements<> &sensor_measurements) {
    // Based on: https://www.movable-type.co.uk/scripts/latlong.html
    double rect_dist = sqrt(position.x * position.x + position.y * position.y);
    sensor_measurements.latitude = asin(sin(STARTING_COORDINATES[0]) * cos(rect_dist / EARTH_RADIUS) +
                                        cos(STARTING_COORDINATES[0]) * sin(rect_dist / EARTH_RADIUS) *
                                        position[1] / rect_dist);
    sensor_measurements.longitude = STARTING_COORDINATES[1] + atan2(position[0] / rect_dist * sin(rect_dist / EARTH_RADIUS) *
                                            cos(STARTING_COORDINATES[0]),
                                            cos(rect_dist / EARTH_RADIUS) -
                                            sin(STARTING_COORDINATES[0]) * sin(sensor_measurements.latitude));
}

SensorMeasurements<> getSensorMeasurements(const Vector<n> &state, const Accel<> &accel) {
    Quaternion<> quat{state[KF::q0], state[KF::q1], state[KF::q2], state[KF::q3]};
    Vector3<> position{state[KF::px], state[KF::py], state[KF::pz]};
    Vector3<> velocity{state[KF::vx], state[KF::vy], state[KF::vz]};
    double altitude = -position.z;

    SensorMeasurements sensor_measurements;
    altitudeSensor(altitude, sensor_measurements);
    gps(position, sensor_measurements);
    imu(quat, accel.linear, accel.angular,
        Vector3<>{state[KF::wx], state[KF::wy], state[KF::wz]},
        Vector3<>{state[KF::accel_bx], state[KF::accel_by], state[KF::accel_bz]},
        Vector3<>{state[KF::gyro_bx], state[KF::gyro_by], state[KF::gyro_bz]},
        sensor_measurements);
    railDetection(quat, altitude, sensor_measurements);
    opticalFlow(velocity, altitude, sensor_measurements);

    return sensor_measurements;
}

void getSensorMeasurementsWrapper(const double *aircraft_state, double *double_sensor_measurements,
                                  uint8_t *uint8_sensor_measurements, unsigned char *bool_sensor_measurements) {
    Vector<n> aircraft_state_vec;

    // only copy over the position, orientation, velocity, and angular velocity
    // TODO: clean up
    for (size_t i = 0; i < 13; i++) aircraft_state_vec[i] = aircraft_state[i];

    Wrench<> applied_loads = AppliedLoads{}.getAppliedLoads(aircraft_state_vec);

    getSensorMeasurements(aircraft_state_vec, toAccel(applied_loads)).assignZ(double_sensor_measurements,
                                                                              uint8_sensor_measurements,
                                                                              bool_sensor_measurements);
}

std::pair<Matrix<p, n>, Matrix<p, 6>> getSensorMeasurementsJacobian(const Vector<n> &state, const Accel<> &accel) {
    Matrix<p, n> h_jac = Matrix<p, n>::zeros();
    Matrix<p, 6> accel_to_h_jac = Matrix<p, 6>::zeros();

    // TODO
    h_jac[SensorMeasurements<>::PRESSURE][KF::px] = AIR_DENSITY * GRAVITATIONAL_ACCELERATION;

    h_jac[SensorMeasurements<>::IMU_wx][KF::wx] = 1;
    h_jac[SensorMeasurements<>::IMU_wy][KF::wy] = 1;
    h_jac[SensorMeasurements<>::IMU_wz][KF::wz] = 1;

    h_jac[SensorMeasurements<>::ALT][KF::pz] = -1;

    return {h_jac, accel_to_h_jac};
}
