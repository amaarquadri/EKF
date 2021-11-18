#pragma once

#include <array>
#include "Vector.h"
#include "Matrix.h"
#include <functional>

template<size_t n, size_t m, size_t p, typename T = double>
class Matrix3D {
protected:
    std::array<Matrix<m, p, T>, n> data{};
public:
    Matrix3D() = default;

    static Matrix3D<n, m, p, T> zeros() {
        return {};
    }

    Vector<n * m * p, T> flatten() const {
        Vector<n * m * p, T> flat_mat;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            flat_mat[i * m * p + j * p + k] = data[i][j][k];
        return flat_mat;
    }

    template<typename R>
    Matrix3D<n, m, p, R> applyFunc(const std::function<R(const T &)> &func) const {
        Matrix3D<n, m, p, R> result;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            result[i][j][k] = func(data[i][j][k]);
        return result;
    }

    Matrix3D<n, m, p, T> operator+(const Matrix3D<n, m, p, T> &other) const {
        Matrix3D<n, m, p, T> sum;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            sum[i][j][k] = data[i][j][k] + other[i][j][k];
        return sum;
    }

    void operator+=(const Matrix3D<n, m, p, T> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            data[i][j][k] += other[i][j][k];
    }

    Matrix3D<n, m, p, T> operator-(const Matrix3D<n, m, p, T> &other) const {
        Matrix3D<n, m, p, T> difference;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            difference[i][j][k] = data[i][j][k] - other[i][j][k];
        return difference;
    }

    void operator-=(const Matrix3D<n, m, p, T> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            data[i][j][k] -= other[i][j][k];
    }

    Matrix3D<n, m, p, T> operator*(const T &scalar) const {
        Matrix3D<n, m, p, T> product;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            product[i][j][k] = data[i][j][k] * scalar;
        return product;
    }

    void operator*=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++) data[i][j][k] *= scalar;
    }

    Matrix3D<n, m, p, T> operator/(const T &scalar) const {
        Matrix3D<n, m, p, T> quotient;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            quotient[i][j][k] = data[i][j][k] / scalar;
        return quotient;
    }

    void operator/=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
            data[i][j][k] /= scalar;
    }

    Matrix<n, m, T> operator*(const Vector<p, T> &vec) const {
        Matrix<n, m, T> product{};
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) for (size_t k = 0; k < p; k++)
                product[i][j] += data[i][j][k] * vec[k];
        return product;
    }

    Matrix<m, p, T> &operator[](const size_t &index) {
        return this->data[index];
    }

    const Matrix<m, p, T> &operator[](const size_t &index) const {
        return this->data[index];
    }
};
