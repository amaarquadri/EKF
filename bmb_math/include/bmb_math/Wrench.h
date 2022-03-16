#pragma once

#include <bmb_math/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <cstddef>

template <typename T>
struct Wrench {
  Vector3<T> force;
  Vector3<T> torque;

  constexpr Wrench(const T& fx = 0, const T& fy = 0, const T& fz = 0,
                   const T& tx = 0, const T& ty = 0, const T& tz = 0) {
    // TODO: figure out how to use initializer list to delegate to the constexpr
    //  Vector3 constructor in a constexpr way
    force.x = fx;
    force.y = fy;
    force.z = fz;
    torque.x = tx;
    torque.y = ty;
    torque.z = tz;
  }

  constexpr Wrench(const Vector3<T>& force,
                   const Vector3<T>& torque = Vector3<T>{})
      : force(force), torque(torque) {}

  constexpr Wrench(const geometry_msgs::Wrench& msg)
      : force(msg.force), torque(msg.torque) {}

  constexpr void copy_to(geometry_msgs::Wrench& msg) const {
    force.copy_to(msg.force);
    torque.copy_to(msg.torque);
  }

  constexpr Wrench<T> operator+(const Wrench<T>& other) const {
    return {Vector3<T>{force + other.force}, Vector3<T>{torque + other.torque}};
  }

  constexpr Wrench<T> operator-(const Wrench<T>& other) const {
    return {Vector3<T>{force - other.force}, Vector3<T>{torque - other.torque}};
  }

  constexpr Wrench<T> operator*(const Wrench<T>& other) const {
    // elementwise multiplication
    return {Vector3<T>{force * other.force}, Vector3<T>{torque * other.torque}};
  }

  constexpr Wrench<T> operator/(const Wrench<T>& other) const {
    // elementwise division
    return {Vector3<T>{force / other.force}, Vector3<T>{torque / other.torque}};
  }

  constexpr Wrench<T> operator+(const T& scalar) const {
    return {Vector3<T>{force + scalar}, Vector3<T>{torque + scalar}};
  }

  constexpr Wrench<T> operator-(const T& scalar) const {
    return {Vector3<T>{force - scalar}, Vector3<T>{torque - scalar}};
  }

  constexpr Wrench<T> operator*(const T& scalar) const {
    return {Vector3<T>{force * scalar}, Vector3<T>{torque * scalar}};
  }

  constexpr Wrench<T> operator/(const T& scalar) const {
    return {Vector3<T>{force / scalar}, Vector3<T>{torque / scalar}};
  }

  constexpr void operator+=(const Wrench<T>& other) {
    force += other.force;
    torque += other.torque;
  }

  constexpr void operator-=(const Wrench<T>& other) {
    force -= other.force;
    torque -= other.torque;
  }

  constexpr void operator*=(const Wrench<T>& other) {
    // elementwise multiplication
    force *= other.force;
    torque *= other.torque;
  }

  constexpr void operator/=(const Wrench<T>& other) {
    // elementwise division
    force /= other.force;
    torque /= other.torque;
  }

  constexpr void operator+=(const T& scalar) {
    force += scalar;
    torque += scalar;
  }

  constexpr void operator-=(const T& scalar) {
    force -= scalar;
    torque -= scalar;
  }

  constexpr void operator*=(const T& scalar) {
    force *= scalar;
    torque *= scalar;
  }

  constexpr void operator/=(const T& scalar) {
    force /= scalar;
    torque /= scalar;
  }

  constexpr T& operator[](const size_t& index) {
    return index < 3 ? force[index] : torque[index - 3];
  }

  constexpr const T& operator[](const size_t& index) const {
    return index < 3 ? force[index] : torque[index - 3];
  }
};

template <typename T>
constexpr Wrench<T> operator+(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar + wrench.force, scalar + wrench.torque};
}

template <typename T>
constexpr Wrench<T> operator-(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar - wrench.force, scalar - wrench.torque};
}

template <typename T>
constexpr Wrench<T> operator*(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar * wrench.force, scalar * wrench.torque};
}

template <typename T>
constexpr Wrench<T> operator/(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar / wrench.force, scalar / wrench.torque};
}
