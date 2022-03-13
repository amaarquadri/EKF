#pragma once

#include <bmb_math/Vector.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

template <typename T>
class Vector3 : public Vector<T, 3> {
 public:
  T& x = this->data[0];
  T& y = this->data[1];
  T& z = this->data[2];

 public:
  Vector3(T x = 0, T y = 0, T z = 0) {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  Vector3(const Vector3<T>& other) : Vector3(other.x, other.y, other.z) {}

  Vector3& operator=(const Vector3<T>& other) {
    // need to overload this operator to allow copying the contents of a Vector3
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }

  // allow implicit conversions
  Vector3(const Vector<T, 3>& vec)
      : Vector3(vec[0], vec[1], vec[2]) {
  }  // NOLINT(google-explicit-constructor)

  Vector3(const geometry_msgs::Vector3& msg)
      : Vector3(msg.x, msg.y, msg.z) {}  // NOLINT(google-explicit-constructor)

  void copy_to(geometry_msgs::Vector3& msg) const {
    msg.x = x;
    msg.y = y;
    msg.z = z;
  }

  Vector3(const geometry_msgs::Point& msg)
      : Vector3(msg.x, msg.y, msg.z) {}  // NOLINT(google-explicit-constructor)

  void copy_to(geometry_msgs::Point& msg) const {
    msg.x = x;
    msg.y = y;
    msg.z = z;
  }

  Vector3<T> cross(const Vector3<T>& other) const {
    return Vector3{y * other.z - z * other.y, z * other.x - x * other.z,
                   x * other.y - y * other.x};
  }
};
