#include "AircraftState.h"

Vector<19> AircraftState::getX() const {
    return Vector<19>{
            position.x, position.y, position.z,
            orientation.q0, orientation.q1, orientation.q2, orientation.q3,
            body_velocity.x, body_velocity.y, body_velocity.z,
            body_angular_velocity.x, body_angular_velocity.y, body_angular_velocity.z,
            body_angular_acceleration.x, body_angular_acceleration.y, body_angular_acceleration.z,
            body_acceleration.x, body_acceleration.y, body_acceleration.z};
}
