#pragma once

#include <bmb_math/Vector.h>

#include <array>
#include <cmath>
#include <stdexcept>

template<typename T, size_t n, size_t m>
class Matrix {
protected:
    std::array<Vector<T, m>, n> data{};
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

    static Matrix<T, n, m> identity() {
        Matrix<T, n, m> output{};
        for (size_t i = 0; i < n && i < m; i++) output[i][i] = 1;
        return output;
    }

    static Matrix<T, n, m> zeros() {
        return {};
    }

    Matrix<T, m, n> transpose() const {
        Matrix<T, m, n> transpose;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) transpose[j][i] = data[i][j];
        return transpose;
    }


    Vector<T, n * m> flatten() const {
        Vector<T, n * m> flat_mat;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) flat_mat[i * m + j] = data[i][j];
        return flat_mat;
    }

    // conceptually this function should be an instance method of the Vector class,
    // but that is not possible due to the resulting circular dependence between Vector and Matrix
    static Matrix<T, n, m> outerProduct(const Vector<T, n> &first, const Vector<T, m> &second) {
        Matrix<T, n, m> outerProduct;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < n; j++)
            outerProduct[i][j] = first[i] * second[j];
        return outerProduct;
    }

    template<typename R>
    Matrix<R, n, m> applyFunc(const std::function<R(const T &)> &func) const {
        Matrix<R, n, m> result;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) result[i][j] = func(data[i][j]);
        return result;
    }

    Matrix<T, m, n> cholesky() const {
        if (m != n) throw std::invalid_argument("Cannot find the Cholesky Decomposition of a non-square matrix");
        Matrix<T, m, n> L; // Lower-triangular matrix defined by A=LL'
        for (size_t i = 0; i < m; i++) {
            for (size_t j = 0; j <= i; j++) {
                T sum{};
                for (size_t k = 0; k < j; k++)
                    sum += L[i][k] * L[j][k];

                if (i == j)
                    L[i][j] = sqrt(this->data[i][i] - sum);
                else
                    L[i][j] = (static_cast<T>(1) / L[j][j] * (this->data[i][j] - sum));
            }
        }
        return L;
    }

    Matrix<T, m, n> inv() const {
        if (m != n) throw std::invalid_argument("Cannot find the inverse of a non-square matrix");
        Matrix<T, m, n> L = cholesky();
        Matrix<T, m, n> u;
        Matrix<T, m, n> I = Matrix<T, m, n>::identity();

        // forward substitution
        for (size_t k = 0; k < n; k++) {
            for (size_t i = 0; i < m; i++) {
                T alpha = I[i][k];
                for (size_t j = 0; j < i; j++) {
                    alpha -= L[i][j] * u[i][j];
                }
                u[i][k] = alpha / L[i][i];
            }
        }

        return u.transpose() * u;
    }

    Matrix<T, n, m> operator+(const Matrix<T, n, m> &other) const {
        Matrix<T, n, m> sum;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) sum[i][j] = data[i][j] + other[i][j];
        return sum;
    }

    void operator+=(const Matrix<T, n, m> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] += other[i][j];
    }

    Matrix<T, n, m> operator-(const Matrix<T, n, m> &other) const {
        Matrix<T, n, m> difference;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) difference[i][j] = data[i][j] - other[i][j];
        return difference;
    }

    void operator-=(const Matrix<T, n, m> &other) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] -= other[i][j];
    }

    Matrix<T, n, m> operator*(const T &scalar) const {
        Matrix<T, n, m> product;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) product[i][j] = data[i][j] * scalar;
        return product;
    }

    void operator*=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] *= scalar;
    }

    Matrix<T, n, m> operator/(const T &scalar) const {
        Matrix<T, n, m> quotient;
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) quotient[i][j] = data[i][j] / scalar;
        return quotient;
    }

    void operator/=(const T &scalar) {
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++) data[i][j] /= scalar;
    }

    template<size_t p>
    Matrix<T, n, p> operator*(const Matrix<T, m, p> &other) const {
        Matrix<T, n, p> product{};
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < p; j++)
            for (size_t k = 0; k < m; k++) product[i][j] += data[i][k] * other[k][j];
        return product;
    }

    Vector<T, n> operator*(const Vector<T, m> &vec) const {
        Vector<T, n> product{};
        for (size_t i = 0; i < n; i++) for (size_t j = 0; j < m; j++)
            product[i] += data[i][j] * vec[j];
        return product;
    }

    Vector<T, m> &operator[](const size_t &index) {
        return this->data[index];
    }

    const Vector<T, m> &operator[](const size_t &index) const {
        return this->data[index];
    }
};
