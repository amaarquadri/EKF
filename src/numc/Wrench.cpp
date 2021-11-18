#include "Wrench.h"
#include "Constants.h"


Accel<> toAccel(const Wrench<> &wrench) {
    return {wrench.force / MASS, INERTIA_TENSOR_INV * wrench.torque};
}
