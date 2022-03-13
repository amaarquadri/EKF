#pragma once

#include <bmb_utilities/MathUtils.h>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <sstream>
#include <string>

template <typename T, size_t n>
class Vector {
 public:
  using iterator = typename std::array<T, n>::iterator;
  using const_iterator = typename std::array<T, n>::const_iterator;

 protected:
  std::array<T, n> data{};

 public:
  Vector() = default;

  Vector(std::initializer_list<T> elements) {
    // Function caller must ensure the number of arguments matches the template
    // argument Excess arguments will be ignored
    size_t i = 0;
    for (auto it = elements.begin(); i < n && it != elements.end(); it++)
      data[i++] = *it;
  }

  explicit Vector(const T* arr) {
    // initializes this Vector with data from the raw C++ array
    // this should only be used when inputting data from MATLAB
    for (size_t i = 0; i < n; i++) data[i] = arr[i];
  }

  Vector<T, n>& operator=(const Vector<T, n>& other) {
    for (size_t i = 0; i < n; i++) data[i] = other[i];
    return *this;
  }

  Vector(const Vector<T, n>& other) { *this = other; }

  template <typename OStream>
  void toCSV(OStream& out) const {
    for (int i = 0; i < n; i++) out << data[i] << std::endl;
  }

  T magnitudeSquared() const {
    T sum{};
    for (size_t i = 0; i < n; i++) sum += data[i] * data[i];
    return sum;
  }

  T magnitude() const { return sqrt(magnitudeSquared()); }

  void normalize() {
    const T size = this->magnitude();
    for (size_t i = 0; i < n; i++) data[i] /= size;
  }

  T dot(const Vector<T, n>& other) const {
    T sum{};
    for (size_t i = 0; i < n; i++) sum += data[i] * other[i];
    return sum;
  }

  template <size_t m>
  Vector<T, n + m> concatenate(const Vector<T, m>& other) const {
    Vector<T, n + m> concat;
    for (size_t i = 0; i < n; i++) concat[i] = data[i];
    for (size_t i = n; i < n + m; i++) concat[i] = other[i];
    return concat;
  }

  template <size_t start = 0, size_t stop = n, size_t step = 1>
  Vector<T, bmb_utilities::slice_count(start, stop, step)> slice() const {
    static_assert(stop <= n);
    static constexpr size_t m = bmb_utilities::slice_count(start, stop, step);
    Vector<T, m> vec;
    for (size_t i = 0; i < m; i++) vec[i] = data[start + i * step];
    return vec;
  }

  template <size_t start = 0, size_t step = 1, size_t m>
  void pasteSlice(const Vector<T, m>& vec) {
    static constexpr size_t stop = start + step * (m - 1) + 1;
    static_assert(stop <= n);
    for (size_t i = 0; i < m; i++) data[start + i * step] = vec[i];
  }

  template <typename R>
  Vector<R, n> applyFunc(const std::function<R(const T&)>& func) const {
    Vector<R, n> result;
    for (size_t i = 0; i < n; i++) result[i] = func(data[i]);
    return result;
  }

  Vector<T, n + 1> pushFront(const T& value) const {
    Vector<T, n + 1> result;
    result[0] = value;
    for (size_t i = 0; i < n; i++) result[i + 1] = data[i];
    return result;
  }

  Vector<T, n + 1> pushBack(const T& value) const {
    Vector<T, n + 1> result;
    for (size_t i = 0; i < n; i++) result[i] = data[i];
    result[n] = value;
    return result;
  }

  Vector<T, n - 1> popFront() const {
    assert(n > 0);
    return slice<1>();
  }

  Vector<T, n - 1> popBack() const {
    assert(n > 0);
    return slice<0, n - 1>();
  }

  Vector<T, n> pushFrontPopBack(const T& value) const {
    return pushFront(value).popBack();
  }

  Vector<T, n> pushBackPopFront(const T& value) const {
    return pushBack(value).popFront();
  }

  Vector<T, n> operator+(const Vector<T, n>& other) const {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++) sum[i] = data[i] + other[i];
    return sum;
  }

  template <size_t start = 0, size_t step = 1, size_t m>
  void operator+=(const Vector<T, m>& other) {
    static constexpr size_t stop = start + step * (m - 1) + 1;
    static_assert(stop <= n);
    for (size_t i = 0; i < m; i++) data[start + step * i] += other[i];
  }

  Vector<T, n> operator+(const double& scalar) const {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++) sum[i] = data[i] + scalar;
    return sum;
  }

  /**
   * If scalar overlaps with any of the elements of this Vector,
   * then the result is undefined behaviour.
   */
  void operator+=(const double& scalar) {
    for (size_t i = 0; i < n; i++) data[i] += scalar;
  }

  Vector<T, n> operator-() const {
    Vector<T, n> negative;
    for (size_t i = 0; i < n; i++) negative[i] = -data[i];
    return negative;
  }

  Vector<T, n> operator+() const {
    Vector<T, n> positive = *this;
    return positive;
  }

  Vector<T, n> operator-(const Vector<T, n>& other) const {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++) sum[i] = data[i] - other[i];
    return sum;
  }

  template <size_t start = 0, size_t step = 1, size_t m>
  void operator-=(const Vector<T, m>& other) {
    static constexpr size_t stop = start + step * (m - 1) + 1;
    static_assert(stop <= n);
    for (size_t i = 0; i < m; i++) data[start + step * i] -= other[i];
  }

  Vector<T, n> operator-(const double& scalar) {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++) sum[i] = data[i] - scalar;
    return sum;
  }

  /**
   * If scalar overlaps with any of the elements of this Vector,
   * then the result is undefined behaviour.
   */
  void operator-=(const double& scalar) {
    for (size_t i = 0; i < n; i++) data[i] -= scalar;
  }

  Vector<T, n> operator*(const T& scalar) const {
    Vector<T, n> product;
    for (size_t i = 0; i < n; i++)
      product[i] = data[i] * scalar;  // respect operator order in case the
                                      // underlying type is non-commutative
    return product;
  }

  /**
   * If scalar overlaps with any of the elements of this Vector,
   * then the result is undefined behaviour.
   */
  void operator*=(const T& scalar) {
    for (size_t i = 0; i < n; i++) data[i] *= scalar;
  }

  Vector<T, n> operator*(const Vector<T, n>& other) const {
    // elementwise multiplication
    Vector<T, n> product;
    for (size_t i = 0; i < n; i++) product[i] = data[i] * other[i];
    return product;
  }

  void operator*=(const Vector<T, n>& other) {
    // elementwise multiplication
    for (size_t i = 0; i < n; i++) data[i] *= other[i];
  }

  Vector<T, n> operator/(const T& scalar) const {
    Vector<T, n> product;
    for (size_t i = 0; i < n; i++) product[i] = data[i] / scalar;
    return product;
  }

  /**
   * If scalar overlaps with any of the elements of this Vector,
   * then the result is undefined behaviour.
   */
  void operator/=(const T& scalar) {
    for (size_t i = 0; i < n; i++) data[i] /= scalar;
  }

  Vector<T, n> operator/(const Vector<T, n>& other) const {
    // elementwise division
    Vector<T, n> quotient;
    for (size_t i = 0; i < n; i++) quotient[i] = data[i] / other[i];
    return quotient;
  }

  void operator/=(const Vector<T, n>& other) {
    // elementwise division
    for (size_t i = 0; i < n; i++) data[i] /= other[i];
  }

  T& operator[](const size_t& index) { return this->data[index]; }

  const T& operator[](const size_t& index) const { return this->data[index]; }

  iterator begin() { return data.begin(); }

  iterator end() { return data.end(); }

  const_iterator begin() const { return data.begin(); }

  const_iterator end() const { return data.end(); }

  const_iterator cbegin() const { return data.begin(); }

  const_iterator cend() const { return data.end(); }

  [[nodiscard]] std::string toStr() const {
    if (n == 0) return "{}";
    std::stringstream out;
    out << "{" << data[0];
    for (size_t i = 1; i < n; i++) out << ", " << data[i];
    out << "}";
    return out.str();
  }
};

template <typename T, size_t n>
Vector<T, n> operator+(const T& scalar, const Vector<T, n>& vec) {
  Vector<T, n> sum;
  for (size_t i = 0; i < n; i++)
    sum[i] = scalar + vec[i];  // respect operator order in case underlying type
                               // is non-commutative
  return sum;
}

template <typename T, size_t n>
Vector<T, n> operator-(const T& scalar, const Vector<T, n>& vec) {
  Vector<T, n> sum;
  for (size_t i = 0; i < n; i++)
    sum[i] = scalar - vec[i];  // respect operator order in case underlying type
                               // is non-commutative
  return sum;
}

template <typename T, size_t n>
Vector<T, n> operator*(const T& scalar, const Vector<T, n>& vec) {
  Vector<T, n> sum;
  for (size_t i = 0; i < n; i++)
    sum[i] = scalar * vec[i];  // respect operator order in case underlying type
                               // is non-commutative
  return sum;
}

template <typename T, size_t n>
Vector<T, n> operator/(const T& scalar, const Vector<T, n>& vec) {
  Vector<T, n> sum;
  for (size_t i = 0; i < n; i++)
    sum[i] = scalar / vec[i];  // respect operator order in case underlying type
                               // is non-commutative
  return sum;
}
