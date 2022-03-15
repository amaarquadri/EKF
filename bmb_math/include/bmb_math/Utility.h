#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Vector.h>
#include <cmath>
#include <cstddef>

namespace bmb_math {

// matrices cannot be declared constexpr
template <typename T>
const Matrix<T, 2, 2> ROT_90_CW{0, 1, -1, 0};
template <typename T>
const Matrix<T, 2, 2> ROT_90_CCW{0, -1, 1, 0};

template <typename T>
Matrix<T, 2, 2> getRotationMatrix(const T& theta) {
  const T sin = std::sin(theta);
  const T cos = std::cos(theta);
  return {cos, -sin, sin, cos};
}

/**
 * @brief Computes a rotation matrix that would rotate Vector<T, 2>{1, 0} to
 * point in the same direction as the given vec
 */
template <typename T>
Matrix<T, 2, 2> getRotationMatrix(const Vector<T, 2>& vec) {
  const T hypot = vec.magnitude();
  const T sin = vec[1] / hypot;
  const T cos = vec[0] / hypot;
  return {cos, -sin, sin, cos};
}

/**
 * Normalizes the given angle in radians to [0, 2 * pi)
 */
template <typename T>
T normalizeAngle(T angle) {
  while (angle > 2 * M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle;
}

/**
 * Computes the angle of the provided Vector<T, 2> from the positive x axis, in
 * radians.
 */
template <typename T>
T atan2(const Vector<T, 2>& vec) {
  return std::atan2(vec[1], vec[0]);
}

/**
 * Creates a Vector<T, 2> with the provided magnitude and angle from the
 * positive x axis, in radians.
 */
template <typename T>
Vector<T, 2> polarToVec(const T& magnitude, const T& angle) {
  return Vector<T, 2>{magnitude * std::cos(angle), magnitude * std::sin(angle)};
}

}  // namespace bmb_math
