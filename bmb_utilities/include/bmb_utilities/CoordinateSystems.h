#pragma once

#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>
#include <bmb_math/Wrench.h>

namespace bmb_utilities {

template <typename T>
Vector3<T> NEDToNWU(const Vector3<T>& vec) {
  return {vec.x, -vec.y, -vec.z};
}

template <typename T>
Vector3<T> NWUToNED(const Vector3<T>& vec) {
  return {vec.x, -vec.y, -vec.z};
}

template <typename T>
Quaternion<T> NEDToNWU(const Quaternion<T>& quat) {
  // negate the y and z axis components
  return Quaternion<T>{quat.q0, quat.q1, -quat.q2, -quat.q3};
}

template <typename T>
Quaternion<T> NWUToNED(const Quaternion<T>& quat) {
  // negate the y and z axis components
  return Quaternion<T>{quat.q0, quat.q1, -quat.q2, -quat.q3};
}

template <typename T>
Wrench<T> NEDToNWU(const Wrench<T>& wrench) {
  return {NEDToNWU(wrench.force), NEDToNWU(wrench.torque)};
}

template <typename T>
Wrench<T> NWUToNED(const Wrench<T>& wrench) {
  return {NWUToNED(wrench.force), NWUToNED(wrench.torque)};
}

}  // namespace bmb_utilities
