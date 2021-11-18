#pragma once

#include "Expression.h"

namespace utils {
    template<typename T = double>
    T saturation(const T &value, const T &limit) {
        if (value > limit) return limit;
        if (value < -limit) return -limit;
        return value;
    }

    template<typename T = double>
    T saturation(const T &value, const T &min, const T &max) {
        if (value > max) return max;
        if (value < min) return min;
        return value;
    }
}
