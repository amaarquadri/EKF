#include "BasicSensorFilter.h"

BasicSensorFilter::BasicSensorFilter() {
    state.orientation = Quaternion<>::identity();
}

void BasicSensorFilter::update(const SensorMeasurements<> &sensorMeasurements,
                               const ControlInputs &control_inputs, const double &dt) {
    AircraftState last_state = state;
    Vector3<> w_abs = state.orientation.unrotate(state.body_angular_velocity);

    // TODO: x and y based on GPS
    state.position.z = -sensorMeasurements.gps_altitude;
    state.orientation += state.orientation.E().transpose() * w_abs * (dt / 2);
    state.orientation.normalize();
    state.body_velocity = (state.position - last_state.position) / dt;
    state.body_angular_velocity = sensorMeasurements.imu_angular_velocity;
    state.body_angular_acceleration = (state.body_angular_velocity - last_state.body_angular_velocity) / dt;
    state.body_acceleration = sensorMeasurements.imu_acceleration;
}

AircraftState BasicSensorFilter::getOutput() const {
    return state;
}
