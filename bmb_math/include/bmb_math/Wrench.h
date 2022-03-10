#pragma once

#include <bmb_math/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <cstddef>

template <typename T>
struct Wrench {
  Vector3<T> force;
  Vector3<T> torque;

  Wrench(const T& fx = 0, const T& fy = 0, const T& fz = 0, const T& tx = 0,
         const T& ty = 0, const T& tz = 0)
      : force(Vector3<T>{fx, fy, fz}), torque(Vector3<T>{tx, ty, tz}) {}

  Wrench(const Vector3<T>& force, const Vector3<T>& torque = Vector3<T>{})
      : force(force), torque(torque) {}

  Wrench(const geometry_msgs::Wrench& msg)
      : force(msg.force), torque(msg.torque) {}

  void copy_to(geometry_msgs::Wrench& msg) const {
    force.copy_to(msg.force);
    torque.copy_to(msg.torque);
  }

  Wrench<T> operator+(const Wrench<T>& other) const {
    return {Vector3<T>{force + other.force}, Vector3<T>{torque + other.torque}};
  }

  Wrench<T> operator-(const Wrench<T>& other) const {
    return {Vector3<T>{force - other.force}, Vector3<T>{torque - other.torque}};
  }

  Wrench<T> operator+(const T& scalar) const {
    return {Vector3<T>{force + scalar}, Vector3<T>{torque + scalar}};
  }

  Wrench<T> operator-(const T& scalar) const {
    return {Vector3<T>{force - scalar}, Vector3<T>{torque - scalar}};
  }

  Wrench<T> operator*(const T& scalar) const {
    return {Vector3<T>{force * scalar}, Vector3<T>{torque * scalar}};
  }

  Wrench<T> operator/(const T& scalar) const {
    return {Vector3<T>{force / scalar}, Vector3<T>{torque / scalar}};
  }

  void operator+=(const Wrench<T>& other) {
    force += other.force;
    torque += other.torque;
  }

  void operator-=(const Wrench<T>& other) {
    force -= other.force;
    torque -= other.torque;
  }

  void operator+=(const T& scalar) {
    force += scalar;
    torque += scalar;
  }

  void operator-=(const T& scalar) {
    force -= scalar;
    torque -= scalar;
  }

  void operator*=(const T& scalar) {
    force *= scalar;
    torque *= scalar;
  }

  void operator/=(const T& scalar) {
    force /= scalar;
    torque /= scalar;
  }

  T& operator[](const size_t& index) {
    return index < 3 ? force[index] : torque[index - 3];
  }

  const T& operator[](const size_t& index) const {
    return index < 3 ? force[index] : torque[index - 3];
  }
};

template <typename T>
Wrench<T> operator+(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar + wrench.force, scalar + wrench.torque};
}

template <typename T>
Wrench<T> operator-(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar - wrench.force, scalar - wrench.torque};
}

template <typename T>
Wrench<T> operator*(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar * wrench.force, scalar * wrench.torque};
}

template <typename T>
Wrench<T> operator/(const T& scalar, const Wrench<T>& wrench) {
  // respect operator order in case underlying type is non-commutative
  return {scalar / wrench.force, scalar / wrench.torque};
}
