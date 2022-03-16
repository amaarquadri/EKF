#pragma once

#include <bmb_math/Vector3.h>
#include <geometry_msgs/Accel.h>

template <typename T>
struct Accel {
  Vector3<T> linear{};
  Vector3<T> angular{};

  Accel(const T& lin_x = 0, const T& lin_y = 0, const T& lin_z = 0,
        const T& ang_x = 0, const T& ang_y = 0, const T& ang_z = 0)
      : linear(Vector3<T>{lin_x, lin_y, lin_z}),
        angular(Vector3<T>{ang_x, ang_y, ang_z}) {}

  Accel(const Vector3<T>& linear, const Vector3<T>& angular = Vector3<T>{})
      : linear(linear), angular(angular) {}

  Accel(const geometry_msgs::Accel& msg)
      : linear(msg.linear), angular(msg.angular) {}

  void copy_to(geometry_msgs::Accel& msg) const {
    linear.copy_to(msg.linear);
    angular.copy_to(msg.angular);
  }

  Accel<T> operator+(const Accel<T>& other) const {
    return {Vector3<T>{linear + other.linear},
            Vector3<T>{angular + other.angular}};
  }

  Accel<T> operator-(const Accel<T>& other) const {
    return {Vector3<T>{linear - other.linear},
            Vector3<T>{angular - other.angular}};
  }

  Accel<T> operator*(const Accel<T>& other) const {
    // elementwise multiplication
    return {Vector3<T>{linear * other.linear},
            Vector3<T>{angular * other.angular}};
  }

  Accel<T> operator/(const Accel<T>& other) const {
    // elementwise division
    return {Vector3<T>{linear / other.linear},
            Vector3<T>{angular / other.angular}};
  }

  Accel<T> operator+(const T& scalar) const {
    return {Vector3<T>{linear + scalar}, Vector3<T>{angular + scalar}};
  }

  Accel<T> operator-(const T& scalar) const {
    return {Vector3<T>{linear - scalar}, Vector3<T>{angular - scalar}};
  }

  Accel<T> operator*(const T& scalar) const {
    return {Vector3<T>{linear * scalar}, Vector3<T>{angular * scalar}};
  }

  Accel<T> operator/(const T& scalar) const {
    return {Vector3<T>{linear / scalar}, Vector3<T>{angular / scalar}};
  }

  void operator+=(const Accel<T>& other) {
    linear += other.linear;
    angular += other.angular;
  }

  void operator-=(const Accel<T>& other) {
    linear -= other.linear;
    angular -= other.angular;
  }

  void operator*=(const Accel<T>& other) {
    // elementwise multiplication
    linear *= other.linear;
    angular *= other.angular;
  }

  void operator/=(const Accel<T>& other) {
    // elementwise division
    linear /= other.linear;
    angular /= other.angular;
  }

  void operator+=(const T& scalar) {
    linear += scalar;
    angular += scalar;
  }

  void operator-=(const T& scalar) {
    linear -= scalar;
    angular -= scalar;
  }

  void operator*=(const T& scalar) {
    linear *= scalar;
    angular *= scalar;
  }

  void operator/=(const T& scalar) {
    linear /= scalar;
    angular /= scalar;
  }

  T& operator[](const size_t& index) {
    return index < 3 ? linear[index] : angular[index - 3];
  }

  const T& operator[](const size_t& index) const {
    return index < 3 ? linear[index] : angular[index - 3];
  }
};

template <typename T>
Accel<T> operator+(const T& scalar, const Accel<T>& accel) {
  // respect operator order in case underlying type is non-commutative
  return {scalar + accel.linear, scalar + accel.angular};
}

template <typename T>
Accel<T> operator-(const T& scalar, const Accel<T>& accel) {
  // respect operator order in case underlying type is non-commutative
  return {scalar - accel.linear, scalar - accel.angular};
}

template <typename T>
Accel<T> operator*(const T& scalar, const Accel<T>& accel) {
  // respect operator order in case underlying type is non-commutative
  return {scalar * accel.linear, scalar * accel.angular};
}

template <typename T>
Accel<T> operator/(const T& scalar, const Accel<T>& accel) {
  // respect operator order in case underlying type is non-commutative
  return {scalar / accel.linear, scalar / accel.angular};
}
