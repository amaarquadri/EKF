#pragma once

#include "Vector.h"

struct ReferenceCommand {
    double airspeed;
    double altitude;
    Vector<2> waypoint;
};
