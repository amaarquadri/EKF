#pragma once

#include "Vector3.h"
#include "Quaternion.h"

struct AircraftState {
    // represents the AircraftState as stored in Simulink

    Vector3<> position;
    Quaternion<> orientation;
    Vector3<> body_velocity;
    Vector3<> body_angular_velocity;
    Vector3<> body_angular_acceleration;
    Vector3<> body_acceleration;

    [[nodiscard]] Vector<19> getX() const;
};
