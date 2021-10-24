#include "SensorModels.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "KF.h"
#include <cmath>

static void railDetection(const Quaternion<double> &quat, const double &altitude, SensorMeasurements &sensor_measurements) {
    if (altitude < 0) {
        // cannot see rail if we are underground
        // leave found_rail as false and leave rail_pixel_x, rail_pixel_y, rail_pixel_width, and rail_angle as 0
        return;
    }

    Vector3<double> down_earth = quat.unrotate({0, 0, 1}); // down (i.e. the direction the camera is pointing) in earth coordinates
    if (down_earth.z < 0) {
        // cannot see rail if the plane is upside down
        return;
    }

    // rail offset in body coordinates
    Vector3<double> rail_offset_body = quat.rotate(down_earth * altitude / down_earth.z);

    Vector3<double> rail_pixel_location = rail_offset_body * CAMERA_GAIN / rail_offset_body.z;
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

    Vector3<double> rail_direction_body = quat.rotate(NORTH); // rail direction in body coordinates
    Vector3<double> rail_direction_pixels = rail_direction_body * CAMERA_GAIN / rail_offset_body.z;
    sensor_measurements.rail_angle = std::atan2(rail_direction_pixels.y, rail_direction_pixels.x);
}

static void opticalFlow(const Vector3<double> &velocity, const double &altitude, SensorMeasurements &sensor_measurements) {
    Vector3<double> pixel_velocity = velocity* OPTICAL_FLOW_VELOCITY_GAIN / altitude;
    sensor_measurements.pixel_velocity[0] = pixel_velocity.x;
    sensor_measurements.pixel_velocity[1] = pixel_velocity.y;
}

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state) {
    Quaternion<double> quat{state[KF::q0], state[KF::q1], state[KF::q2], state[KF::q3]};
    Vector3<double> velocity{state[KF::vx], state[KF::vy], state[KF::vz]};
    double altitude = -state[KF::pz];

    SensorMeasurements sensor_measurements;
    railDetection(quat, altitude, sensor_measurements);
    opticalFlow(velocity, altitude, sensor_measurements);

//    Wrench<double> wrench = applied_loads.getAppliedLoads(x);
//    // TODO
//    return SensorMeasurements{ATMOSPHERIC_PRESSURE - AIR_DENSITY * GRAVITATIONAL_ACCELERATION * (-x[pz]),
//                              0, 0, 0, 0, false,
//                              Vector<double, 2>{0, 0},
//                              Vector3<double>{wrench.force / MASS},
//                              Vector3<double>{x[wx], x[wy], x[wz]},
//                              0, 0, -x[pz], 8}
//                              .getZ();
    return sensor_measurements;
}
