#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Vector.h>
#include <array>
#include <functional>

template <typename T, size_t n, size_t m, size_t p>
class Matrix3D {
 protected:
  std::array<Matrix<T, m, p>, n> data{};

 public:
  constexpr Matrix3D() = default;

  static constexpr Matrix3D<T, n, m, p> zeros() { return {}; }

  constexpr bool operator==(const Matrix3D<T, n, m, p>& other) const {
    for (size_t i = 0; i < n; i++)
      if (data[i] != other[i]) return false;
    return true;
  }

  constexpr bool operator!=(const Matrix3D<T, n, m, p>& other) const {
    return !(*this == other);
  }

  [[nodiscard]] constexpr Vector<T, n * m * p> flatten() const {
    Vector<T, n * m * p> flat_mat;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++)
          flat_mat[i * m * p + j * p + k] = data[i][j][k];
    return flat_mat;
  }

  template <typename R>
  Matrix3D<R, n, m, p> applyFunc(const std::function<R(const T&)>& func) const {
    Matrix3D<R, n, m, p> result;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++) result[i][j][k] = func(data[i][j][k]);
    return result;
  }

  constexpr Matrix3D<T, n, m, p> operator+(
      const Matrix3D<T, n, m, p>& other) const {
    Matrix3D<T, n, m, p> sum;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++)
          sum[i][j][k] = data[i][j][k] + other[i][j][k];
    return sum;
  }

  constexpr void operator+=(const Matrix3D<T, n, m, p>& other) {
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++) data[i][j][k] += other[i][j][k];
  }

  constexpr Matrix3D<T, n, m, p> operator-(
      const Matrix3D<T, n, m, p>& other) const {
    Matrix3D<T, n, m, p> difference;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++)
          difference[i][j][k] = data[i][j][k] - other[i][j][k];
    return difference;
  }

  constexpr void operator-=(const Matrix3D<T, n, m, p>& other) {
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++) data[i][j][k] -= other[i][j][k];
  }

  constexpr Matrix3D<T, n, m, p> operator*(const T& scalar) const {
    Matrix3D<T, n, m, p> product;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++)
          product[i][j][k] = data[i][j][k] * scalar;
    return product;
  }

  constexpr void operator*=(const T& scalar) {
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++) data[i][j][k] *= scalar;
  }

  constexpr Matrix3D<T, n, m, p> operator/(const T& scalar) const {
    Matrix3D<T, n, m, p> quotient;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++)
          quotient[i][j][k] = data[i][j][k] / scalar;
    return quotient;
  }

  constexpr void operator/=(const T& scalar) {
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++) data[i][j][k] /= scalar;
  }

  constexpr Matrix<T, n, m> operator*(const Vector<T, p>& vec) const {
    Matrix<T, n, m> product{};
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        for (size_t k = 0; k < p; k++) product[i][j] += data[i][j][k] * vec[k];
    return product;
  }

  constexpr Matrix<T, m, p>& operator[](const size_t& index) {
    return this->data[index];
  }

  constexpr const Matrix<T, m, p>& operator[](const size_t& index) const {
    return this->data[index];
  }
};
