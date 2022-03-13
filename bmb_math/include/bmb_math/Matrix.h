#pragma once

#include <bmb_math/Vector.h>
#include <bmb_utilities/MathUtils.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

template <typename T, size_t n, size_t m>
class Matrix {
 protected:
  std::array<Vector<T, m>, n> data{};

 public:
  Matrix() = default;

  Matrix(std::initializer_list<T> elements) {
    // Function caller must ensure the number of arguments matches the template
    // arguments Excess arguments will be ignored
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

  Matrix<T, n, m>& operator=(const Matrix<T, n, m>& other) {
    // delegate to Vector<T, m>::operator=
    for (size_t i = 0; i < n; i++) data[i] = other[i];
    return *this;
  }

  static Matrix<T, n, m> identity() {
    static constexpr size_t k = std::min(n, m);
    Matrix<T, n, m> output{};
    for (size_t i = 0; i < k; i++) output[i][i] = 1;
    return output;
  }

  static Matrix<T, n, m> zeros() { return {}; }

  Matrix<T, m, n> transpose() const {
    Matrix<T, m, n> transpose;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) transpose[j][i] = data[i][j];
    return transpose;
  }

  void transposeInPlace() {
    if constexpr (n != m)
      throw std::invalid_argument(
          "Cannot transpose in place a non-square matrix");

    for (size_t i = 1; i < n; i++)
      for (size_t j = 0; j < i; j++) std::swap(data[i][j], data[j][i]);
  }

  Vector<T, n * m> flatten() const {
    Vector<T, n * m> flat_mat;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) flat_mat[i * m + j] = data[i][j];
    return flat_mat;
  }

  // conceptually this function should be an instance method of the Vector
  // class, but that is not possible due to the resulting circular dependence
  // between Vector and Matrix
  static Matrix<T, n, m> outerProduct(const Vector<T, n>& first,
                                      const Vector<T, m>& second) {
    Matrix<T, n, m> outerProduct;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) outerProduct[i][j] = first[i] * second[j];
    return outerProduct;
  }

  template <size_t start = 0, size_t stop = n, size_t step = 1>
  Matrix<T, bmb_utilities::slice_count(start, stop, step), m> sliceRows()
      const {
    static_assert(stop <= n);
    static constexpr size_t k = bmb_utilities::slice_count(start, stop, step);
    Matrix<T, k, m> mat;
    for (size_t i = 0; i < k; i++) mat[i] = data[start + i * step];
    return mat;
  }

  template <size_t start = 0, size_t stop = m, size_t step = 1>
  Matrix<T, n, bmb_utilities::slice_count(start, stop, step)> sliceColumns()
      const {
    static_assert(stop <= m);
    static constexpr size_t k = bmb_utilities::slice_count(start, stop, step);
    Matrix<T, n, k> mat;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < k; j++) mat[i][j] = data[i][start + j * step];
    return mat;
  }

  template <size_t start = 0, size_t step = 1, size_t k>
  void pasteSliceRows(const Matrix<T, k, m>& mat) {
    static constexpr size_t stop = start + step * (k - 1) + 1;
    static_assert(stop <= n);
    // delegate to Vector<T, m>::operator=
    for (size_t i = 0; i < k; i++) data[start + i * step] = mat[i];
  }

  template <size_t start = 0, size_t step = 1, size_t k>
  void pasteSliceColumns(const Matrix<T, n, k>& mat) {
    static constexpr size_t stop = start + step * (k - 1) + 1;
    static_assert(stop <= m);
    // delegate to Vector<T, m>::operator=
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < k; j++) data[i][start + j * step] = mat[i][j];
  }

  template <typename R>
  Matrix<R, n, m> applyFunc(const std::function<R(const T&)>& func) const {
    Matrix<R, n, m> result;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) result[i][j] = func(data[i][j]);
    return result;
  }

  Matrix<T, m, n> cholesky() const {
    if constexpr (m != n)
      throw std::invalid_argument(
          "Cannot find the Cholesky Decomposition of a non-square matrix");
    Matrix<T, m, n> L;  // Lower-triangular matrix defined by A=LL'
    for (size_t i = 0; i < m; i++) {
      for (size_t j = 0; j <= i; j++) {
        T sum{};
        for (size_t k = 0; k < j; k++) sum += L[i][k] * L[j][k];

        if (i == j)
          L[i][j] = sqrt(this->data[i][i] - sum);
        else
          L[i][j] = (static_cast<T>(1) / L[j][j] * (this->data[i][j] - sum));
      }
    }
    return L;
  }

  Matrix<T, m, n> inv() const {
    if constexpr (m != n)
      throw std::invalid_argument(
          "Cannot find the inverse of a non-square matrix");
    const Matrix<T, m, n> L = cholesky();
    Matrix<T, m, n> u;

    // forward substitution
    for (size_t k = 0; k < n; k++) {
      for (size_t i = 0; i < m; i++) {
        T alpha = (i == k) ? 1 : 0;
        for (size_t j = 0; j < i; j++) {
          alpha -= L[i][j] * u[i][j];
        }
        u[i][k] = alpha / L[i][i];
      }
    }

    return u.transpose() * u;
  }

  Matrix<T, n, m> operator+(const Matrix<T, n, m>& other) const {
    Matrix<T, n, m> sum;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) sum[i][j] = data[i][j] + other[i][j];
    return sum;
  }

  template <size_t row_start = 0, size_t col_start = 0, size_t row_step = 1,
            size_t col_step = 1, size_t x, size_t y>
  void operator+=(const Matrix<T, x, y>& other) {
    static constexpr size_t row_stop = row_start + row_step * (x - 1) + 1;
    static constexpr size_t col_stop = col_start + col_step * (y - 1) + 1;
    static_assert(row_stop <= n && col_stop <= m);
    for (size_t i = 0; i < x; i++)
      for (size_t j = 0; j < y; j++)
        data[row_start + i * row_step][col_start + j * col_step] += other[i][j];
  }

  Matrix<T, n, m> operator-(const Matrix<T, n, m>& other) const {
    Matrix<T, n, m> difference;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++)
        difference[i][j] = data[i][j] - other[i][j];
    return difference;
  }

  template <size_t row_start = 0, size_t col_start = 0, size_t row_step = 1,
            size_t col_step = 1, size_t x, size_t y>
  void operator-=(const Matrix<T, x, y>& other) {
    static constexpr size_t row_stop = row_start + row_step * (x - 1) + 1;
    static constexpr size_t col_stop = col_start + col_step * (y - 1) + 1;
    static_assert(row_stop <= n && col_stop <= m);
    for (size_t i = 0; i < x; i++)
      for (size_t j = 0; j < y; j++)
        data[row_start + i * row_step][col_start + j * col_step] -= other[i][j];
  }

  Matrix<T, n, m> operator*(const T& scalar) const {
    Matrix<T, n, m> product;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) product[i][j] = data[i][j] * scalar;
    return product;
  }

  void operator*=(const T& scalar) {
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) data[i][j] *= scalar;
  }

  Matrix<T, n, m> operator/(const T& scalar) const {
    Matrix<T, n, m> quotient;
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) quotient[i][j] = data[i][j] / scalar;
    return quotient;
  }

  void operator/=(const T& scalar) {
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) data[i][j] /= scalar;
  }

  template <size_t p>
  Matrix<T, n, p> operator*(const Matrix<T, m, p>& other) const {
    Matrix<T, n, p> product{};
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < p; j++)
        for (size_t k = 0; k < m; k++)
          product[i][j] += data[i][k] * other[k][j];
    return product;
  }

  Vector<T, n> operator*(const Vector<T, m>& vec) const {
    Vector<T, n> product{};
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < m; j++) product[i] += data[i][j] * vec[j];
    return product;
  }

  Vector<T, m>& operator[](const size_t& index) { return this->data[index]; }

  const Vector<T, m>& operator[](const size_t& index) const {
    return this->data[index];
  }
};
