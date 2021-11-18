#pragma once

#include "Vector3.h"
#include "Matrix.h"

template<typename T = double>
class Quaternion : public Vector<4, T> {
public:
    T &q0 = this->data[0];
    T &q1 = this->data[1];
    T &q2 = this->data[2];
    T &q3 = this->data[3];
public:
    explicit Quaternion(T q0 = 1, T q1 = 0, T q2 = 0, T q3 = 0) {
        this->q0 = q0;
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;
    }

    // allow implicit conversions
    Quaternion(Vector<4, T> vec) : Quaternion(vec[0], vec[1], vec[2], vec[3]) {} // NOLINT(google-explicit-constructor)

    Quaternion& operator=(const Quaternion<T> &other) {
        // need to overload this operator to allow copying the contents of a Quaternion
        q0 = other.q0;
        q1 = other.q1;
        q2 = other.q2;
        q3 = other.q3;
        return *this;
    }

    static Quaternion identity() {
        return Quaternion{1, 0, 0, 0};
    }

    Quaternion cong() const {
        return Quaternion{q0, -q1, -q2, -q3};
    }

    Vector3<T> rotate(const Vector3<T> &vec) const {
        Quaternion x_quat = Quaternion{0, vec.x, vec.y, vec.z};
        Quaternion x_rot_quat = this->cong() * x_quat * (*this);
        return Vector3<T>{x_rot_quat.q1,
                          x_rot_quat.q2,
                          x_rot_quat.q3};
    }

    Vector3<T> unrotate(const Vector3<T> &vec) const {
        return this->cong().rotate(vec);
    }

    Matrix<3, 4, T> E() const {
        Matrix<3, 4, T> matrix{};

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

    Matrix<3, 4, T> G() const {
        Matrix<3, 4, T> matrix{};

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

    Matrix<3, 3, T> toDCM() const {
        // https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran
        Matrix<3, 3, T> DCM;
        T q0q0 = q0 * q0;
        T q1q1 = q1 * q1;
        T q2q2 = q2 * q2;
        T q3q3 = q3 * q3;
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

    Quaternion<T> operator*(const Quaternion<T> &other) const {
        return Quaternion<T>{q0 * other.q0 - q1 * other.q1 - q2 * other.q2 - q3 * other.q3,
                             q0 * other.q1 + other.q0 * q1 + q2 * other.q3 - q3 * other.q2,
                             q0 * other.q2 + other.q0 * q2 + q3 * other.q1 - q1 * other.q3,
                             q0 * other.q3 + other.q0 * q3 + q1 * other.q2 - q2 * other.q1};
    }

    void operator*=(const Quaternion<T> &other) {
        *this = (*this) * other;
    }

    Quaternion<T> operator/(const Quaternion<T> &other) const {
        return (*this) * other.cong();
    }

    void operator/=(const Quaternion<T> &other) {
        *this = (*this) * other.cong();
    }
};
