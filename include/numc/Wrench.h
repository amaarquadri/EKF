#pragma once

#include "Vector3.h"
#include "Accel.h"

template<typename T = double>
struct Wrench {
    Vector3<T> force;
    Vector3<T> torque;

    Wrench<T> operator+(const Wrench<T> &other) {
        return {Vector3<T>{force + other.force}, Vector3<T>{torque + other.torque}};
    }

    Wrench<T> operator-(const Wrench<T> &other) {
        return {Vector3<T>{force - other.force}, Vector3<T>{torque - other.torque}};
    }

    Wrench<T> operator*(const T &scalar) {
        return {Vector3<T>{force * scalar}, Vector3<T>{torque * scalar}};
    }

    Wrench<T> operator/(const double &scalar) {
        return {Vector3<T>{force / scalar}, Vector3<T>{torque / scalar}};
    }

    void operator+=(const Wrench<T> &other) {
        force += other.force;
        torque += other.torque;
    }

    void operator-=(const Wrench<T> &other) {
        force -= other.force;
        torque -= other.torque;
    }

    void operator*=(const T &scalar) {
        force *= scalar;
        torque *= scalar;
    }

    void operator/=(const double &scalar) {
        force /= scalar;
        torque /= scalar;
    }

    T &operator[](const size_t &index) {
        return index < 3 ? force[index] : torque[index - 3];
    }

    const T &operator[](const size_t& index) const {
        return index < 3 ? force[index] : torque[index - 3];
    }
};

Accel<> toAccel(const Wrench<> &wrench);

template<typename T>
Wrench<T> operator+(const T &scalar, const Wrench<T> &wrench) {
    // respect operator order in case underlying type is non-commutative
    return {scalar + wrench.force, scalar + wrench.torque};
}

template<typename T>
Wrench<T> operator-(const T &scalar, const Wrench<T> &wrench) {
    // respect operator order in case underlying type is non-commutative
    return {scalar - wrench.force, scalar - wrench.torque};
}

template<typename T>
Wrench<T> operator*(const T &scalar, const Wrench<T> &wrench) {
    // respect operator order in case underlying type is non-commutative
    return {scalar * wrench.force, scalar * wrench.torque};
}

template<typename T>
Wrench<T> operator/(const T &scalar, const Wrench<T> &wrench) {
    // respect operator order in case underlying type is non-commutative
    return {scalar / wrench.force, scalar / wrench.torque};
}
