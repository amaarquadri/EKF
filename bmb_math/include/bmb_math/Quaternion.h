#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Vector3.h>
#include <geometry_msgs/Quaternion.h>

template <typename T>
class Quaternion : public Vector<T, 4> {
 public:
  T& q0 = this->data[0];  // w
  T& q1 = this->data[1];  // x
  T& q2 = this->data[2];  // y
  T& q3 = this->data[3];  // z

 public:
  explicit constexpr Quaternion(const T& q0 = static_cast<T>(1),
                                const T& q1 = static_cast<T>(0),
                                const T& q2 = static_cast<T>(0),
                                const T& q3 = static_cast<T>(0)) {
    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;
  }

  // allow implicit conversions
  constexpr Quaternion(const Vector<T, 4>& vec) {
    // TODO: figure out how to use initializer list to delegate to the constexpr
    //  Vector3 constructor in a constexpr way
    q0 = vec[0];
    q1 = vec[1];
    q2 = vec[2];
    q3 = vec[3];
  }  // NOLINT(google-explicit-constructor)

  // allow implicit conversions
  constexpr Quaternion(const geometry_msgs::Quaternion& msg)
      : Quaternion(msg.w, msg.x, msg.y, msg.z) {
  }  // NOLINT(google-explicit-constructor)

  constexpr void copy_to(geometry_msgs::Quaternion& msg) {
    msg.w = q0;
    msg.x = q1;
    msg.y = q2;
    msg.z = q3;
  }

  constexpr Quaternion<T>& operator=(const Quaternion<T>& other) {
    // need to overload this operator to allow copying the contents of a
    // Quaternion
    q0 = other.q0;
    q1 = other.q1;
    q2 = other.q2;
    q3 = other.q3;
    return *this;
  }

  static constexpr Quaternion<T> identity() { return Quaternion<T>{}; }

  constexpr Quaternion<T> cong() const { return Quaternion{q0, -q1, -q2, -q3}; }

  constexpr Vector3<T> rotate(const Vector3<T>& vec) const {
    Quaternion x_quat = Quaternion{static_cast<T>(0), vec.x, vec.y, vec.z};
    Quaternion x_rot_quat = this->cong() * x_quat * (*this);
    return Vector3<T>{x_rot_quat.q1, x_rot_quat.q2, x_rot_quat.q3};
  }

  constexpr Vector3<T> unrotate(const Vector3<T>& vec) const {
    return this->cong().rotate(vec);
  }

  T getRoll() const {
    const double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    const double cosr_cosp = 1 - 2 * (q0 * q1 + q2 * q3);
    return std::atan2(sinr_cosp, cosr_cosp);
  }

  T getPitch() const {
    const double sinp = 2 * (q0 * q1 - q2 * q3);
    if (std::abs(sinp) >= 1)
      return std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
      return std::asin(sinp);
  }

  T getYaw() const {
    const double siny_cosp = 2 * (q0 * q1 + q2 * q3);
    const double cosy_cosp = 1 - 2 * (q0 * q1 + q2 * q3);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  Vector3<T> euler_angles() const {
    return Vector3<T>{getRoll(), getPitch(), getYaw()};
  }

  constexpr Matrix<T, 3, 4> E() const {
    Matrix<T, 3, 4> matrix{};

    matrix[0][0] = -q1;
    matrix[0][1] = q0;
    matrix[0][2] = -q3;
    matrix[0][3] = q2;

    matrix[1][0] = -q2;
    matrix[1][1] = q3;
    matrix[1][2] = q0;
    matrix[1][3] = -q1;

    matrix[2][0] = -q3;
    matrix[2][1] = -q2;
    matrix[2][2] = q1;
    matrix[2][3] = q0;

    return matrix;
  }

  constexpr Matrix<T, 3, 4> G() const {
    Matrix<T, 3, 4> matrix{};

    matrix[0][0] = -q1;
    matrix[0][1] = q0;
    matrix[0][2] = q3;
    matrix[0][3] = -q2;

    matrix[1][0] = -q2;
    matrix[1][1] = -q3;
    matrix[1][2] = q0;
    matrix[1][3] = q1;

    matrix[2][0] = -q3;
    matrix[2][1] = q2;
    matrix[2][2] = -q1;
    matrix[2][3] = q0;

    return matrix;
  }

  constexpr Matrix<T, 3, 3> toDCM() const {
    // https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran
    Matrix<T, 3, 3> DCM;
    const T q0q0 = q0 * q0;
    const T q1q1 = q1 * q1;
    const T q2q2 = q2 * q2;
    const T q3q3 = q3 * q3;
    DCM[0][0] = q3q3 + q0q0 - q1q1 - q2q2;
    DCM[1][1] = q3q3 - q0q0 + q1q1 - q2q2;
    DCM[2][2] = q3q3 - q0q0 - q1q1 + q2q2;
    DCM[0][1] = 2 * (q0 * q1 + q2 * q3);
    DCM[0][2] = 2 * (q0 * q2 - q1 * q3);
    DCM[1][2] = 2 * (q1 * q2 + q0 * q3);
    DCM[1][0] = -DCM[0][1];
    DCM[2][0] = -DCM[0][2];
    DCM[2][1] = -DCM[1][2];
    return DCM;
  }

  // OPERATOR OVERLOADING:

  constexpr Quaternion<T> operator*(const Quaternion<T>& other) const {
    return Quaternion<T>{
        q0 * other.q0 - q1 * other.q1 - q2 * other.q2 - q3 * other.q3,
        q0 * other.q1 + other.q0 * q1 + q2 * other.q3 - q3 * other.q2,
        q0 * other.q2 + other.q0 * q2 + q3 * other.q1 - q1 * other.q3,
        q0 * other.q3 + other.q0 * q3 + q1 * other.q2 - q2 * other.q1};
  }

  constexpr void operator*=(const Quaternion<T>& other) {
    *this = (*this) * other;
  }

  constexpr Quaternion<T> operator/(const Quaternion<T>& other) const {
    return (*this) * other.cong();
  }

  constexpr void operator/=(const Quaternion<T>& other) {
    *this = (*this) * other.cong();
  }
};
