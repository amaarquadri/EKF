#include "ControlInputs.h"

ControlInputs ControlInputs::parseU(const Vector<4> &inputs) {
    return {inputs[0], inputs[1], inputs[2], inputs[3]};
}

Vector<4> ControlInputs::getU() const {
    return Vector<4>{propeller_voltage, right_aileron_angle, left_aileron_angle, elevator_angle};
}
