#pragma once

#include "Vector.h"
#include <array>
#include <cmath>
#include <stdexcept>

template<size_t n, size_t m, typename T = double>
class Matrix {
protected:
    std::array<Vector<m, T>, n> data{};
public:
    Matrix() = default;

    Matrix(std::initializer_list<T> elements) {
        // Function caller must ensure the number of arguments matches the template arguments
        // Excess arguments will be ignored
        size_t i = 0;
        size_t j = 0;
        for (auto it = elements.begin(); i < n && it != elements.end(); it++) {
            data[i][j] = *it;
            j++;
            if (j == m) {
                j = 0;
                i++;
            }
        }
    }

    static Matrix<n, m, T> identity() {
        Matrix<n, m, T> output{};
        for (size_t i = 0; i < n && i < m; i++) output[i][i] = 1;
        return output;
    }

    static Matrix<n, m, T> zeros() {
        return {};
    }

    Matrix<m, n, T> transpose() const {
        Matrix<m, n, T> transpose;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) transpose[j][i] = data[i][j];
        return transpose;
    }

    Vector<n * m, T> flatten() const {
        Vector<n * m, T> flat_mat;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) flat_mat[i * m + j] = data[i][j];
        return flat_mat;
    }

    // conceptually this function should be an instance method of the Vector class,
    // but that is not possible due to the resulting circular dependence between Vector and Matrix
    static Matrix<n, m, T> outerProduct(const Vector<n, T> &first, const Vector<m, T> &second) {
        Matrix<n, m, T> outerProduct;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < n; j++)
            outerProduct[i][j] = first[i] * second[j];
        return outerProduct;
    }

    template<typename R>
    Matrix<n, m, R> applyFunc(const std::function<R(const T &)> &func) const {
        Matrix<n, m, R> result;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) result[i][j] = func(data[i][j]);
        return result;
    }

    Matrix<m, n, T> cholesky() const {
        if (m != n) throw std::invalid_argument("Cannot find the Cholesky Decomposition of a non-square matrix");
        Matrix<m, n, T> L; // Lower-triangular matrix defined by A=LL'
        for (size_t i = 0; i < m; i++) {
            for (size_t j = 0; j <= i; j++) {
                T sum{};
                for (size_t k = 0; k < j; k++)
                    sum += L[i][k] * L[j][k];

                if (i == j)
                    L[i][j] = sqrt(this->data[i][i] - sum);
                else
                    L[i][j] = (1.0 / L[j][j] * (this->data[i][j] - sum));
            }
        }
        return L;
    }

    Matrix<m, n, T> inv() const {
        if (m != n) throw std::invalid_argument("Cannot find the inverse of a non-square matrix");
        Matrix<m, n, T> L = cholesky();
        Matrix<m, n, T> u;
        Matrix<m, n, T> I = Matrix<m, n, T>::identity();

        // forward substitution
        for (size_t k = 0; k < n; k++) {
            for (size_t i = 0; i < m; i++) {
                T alpha = I[i][k];
                for (size_t j = 0; j < i - 1; j++) {
                    alpha -= L[i][j] * u[i][j];
                }
                u[i][k] = alpha / L[i][i];
            }
        }

        return u.transpose() * u;
    }

    Matrix<n, m, T> operator+(const Matrix<n, m, T> &other) const {
        Matrix<n, m, T> sum;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) sum[i][j] = data[i][j] + other[i][j];
        return sum;
    }

    void operator+=(const Matrix<n, m, T> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] += other[i][j];
    }

    Matrix<n, m, T> operator-(const Matrix<n, m, T> &other) const {
        Matrix<n, m, T> difference;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) difference[i][j] = data[i][j] - other[i][j];
        return difference;
    }

    void operator-=(const Matrix<n, m, T> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] -= other[i][j];
    }

    Matrix<n, m, T> operator*(const T &scalar) const {
        Matrix<n, m, T> product;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) product[i][j] = data[i][j] * scalar;
        return product;
    }

    void operator*=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] *= scalar;
    }

    Matrix<n, m, T> operator/(const T &scalar) const {
        Matrix<n, m, T> quotient;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) quotient[i][j] = data[i][j] / scalar;
        return quotient;
    }

    void operator/=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] /= scalar;
    }

    template<size_t p>
    Matrix<n, p, T> operator*(const Matrix<m, p, T> &other) const {
        Matrix<n, p, T> product{};
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < p; j++)
            for (size_t k = 0; k < m; k++) product[i][j] += data[i][k] * other[k][j];
        return product;
    }

    Vector<n, T> operator*(const Vector<m, T> &vec) const {
        Vector<n, T> product{};
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++)
            product[i] += data[i][j] * vec[j];
        return product;
    }

    Vector<m, T> &operator[](const size_t &index) {
        return this->data[index];
    }

    const Vector<m, T> &operator[](const size_t &index) const {
        return this->data[index];
    }
};

