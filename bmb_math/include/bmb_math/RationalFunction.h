#pragma once

#include <bmb_math/Polynomial.h>
#include <bmb_utilities/ConstexprUtils.h>
#include <bmb_utilities/MathUtils.h>
#include <algorithm>
#include <initializer_list>
#include <iostream>
#include <vector>

template <typename T, size_t n, size_t m>
class RationalFunction {
  static_assert(n != 0 && m != 0,
                "Numerator and denominator must have at least 1 element!");

 protected:
  Polynomial<T, n> numerator;
  Polynomial<T, m> denominator;

  template <typename, size_t, size_t>
  friend class RationalFunction;
  template <typename, size_t, size_t>
  friend class TransferFunction;

 public:
  RationalFunction() = default;

  RationalFunction(std::initializer_list<T> elements) {
    // Function caller must ensure the number of arguments matches the template
    // argument Excess arguments will be ignored
    size_t i = 0;
    for (auto it = elements.begin(); i < n && it != elements.end(); it++)
      numerator[i++] = *it;
    for (auto it = elements.begin() + i; i < m + n && it != elements.end();
         it++) {
      denominator[i - n] = *it;
      i++;
    }
  }

  RationalFunction(const Polynomial<T, n>& numerator,
                   const Polynomial<T, m>& denominator) {
    this->numerator = numerator;
    this->denominator = denominator;
  }

  RationalFunction(const RationalFunction<T, n, m>& other) { (*this) = other; }

  RationalFunction& operator=(const RationalFunction<T, n, m>& other) {
    numerator = other.numerator;
    denominator = other.denominator;
    return (*this);
  }

  void print(const char& independent_var = 's') const {
    numerator.print(independent_var);
    for (size_t i = 0; i < std::max(n, m); i++) {
      std::cout << "-----";
    }
    std::cout << '\n';
    denominator.print(independent_var);
  }

  T& numerator_data(const size_t& num_index) { return numerator[num_index]; }

  T& denominator_data(const size_t& den_index) {
    return denominator[den_index];
  }

  const T& numerator_data(const size_t& num_index) const {
    return numerator[num_index];
  }

  const T& denominator_data(const size_t& den_index) const {
    return denominator[den_index];
  }

  void normalize() {
    // divide num and den by highest order denominator coefficient
    const double& factor = denominator[m - 1];
    if (factor != 0) {
      numerator /= denominator[m - 1];
      denominator /= denominator[m - 1];
    }
  }

  template <size_t p>
  RationalFunction<T, (n - 1) * (p - 1) + 1, (m - 1) * (p - 1) + 1> _of_(
      const Polynomial<T, p>& p_of_x) const {
    return {numerator._of_(p_of_x), denominator._of_(p_of_x)};
  }

  template <size_t p, size_t q>
  using composite_rational = RationalFunction<
      T,
      (n - 1) * std::max(p - 1, q - 1) +
          (q - 1) * bmb_utilities::heaviside_difference(m, n) + 1,
      (m - 1) * std::max(p - 1, q - 1) +
          (q - 1) * bmb_utilities::heaviside_difference(n, m) + 1>;

  template <size_t p, size_t q>
  composite_rational<p, q> _of_(const RationalFunction<T, p, q>& g_of_x) const {
    Polynomial<T, (n - 1) * std::max(p - 1, q - 1) + 1> num;
    Polynomial<T, (m - 1) * std::max(p - 1, q - 1) + 1> den;

    bmb_utilities::constexprFor<0, n>([&](auto i) {
      num += numerator[i] * g_of_x.numerator.template pow<i>() *
             g_of_x.denominator.template pow<n - 1 - i>();
    });
    bmb_utilities::constexprFor<0, m>([&](auto i) {
      den += denominator[i] * g_of_x.numerator.template pow<i>() *
             g_of_x.denominator.template pow<m - 1 - i>();
    });

    if constexpr (n != m) {  // because the denominators of the numerator and
                             // denominator are the same, they will cancel
      static constexpr size_t abs_diff = bmb_utilities::abs_difference(m, n);
      const Polynomial<T, abs_diff + 1> canceled_denominators =
          g_of_x.denominator.template pow<abs_diff>();
      if constexpr (m > n)
        return {num * canceled_denominators, den};
      else
        return {num, den * canceled_denominators};
    } else
      return {num, den};
  }

  template <size_t p, size_t q>
  RationalFunction<T, n + p - 1, m + q - 1> operator*(
      const RationalFunction<T, p, q>& other) const {
    return {numerator * other.numerator, denominator * other.denominator};
  }

  template <size_t p>
  RationalFunction<T, n + p - 1, m> operator*(
      const Polynomial<T, p>& poly) const {
    return {poly * numerator, denominator};
  }

  RationalFunction<T, n, m> operator*(const T& scalar) const {
    return {scalar * numerator, denominator};
  }

  void operator*=(const T& scalar) { numerator *= scalar; }

  template <size_t p, size_t q>
  RationalFunction<T, n + q - 1, m + p - 1> operator/(
      const RationalFunction<T, p, q>& other) const {
    return {numerator * other.denominator, denominator * other.numerator};
  }

  template <size_t p>
  RationalFunction<T, n, m * p> operator/(const Polynomial<T, p>& poly) const {
    return {numerator, poly * denominator};
  }

  RationalFunction<T, n, m> operator/(const T& scalar) const {
    return {numerator, scalar * denominator};
  }

  void operator/=(const T& scalar) { denominator /= scalar; }

  template <size_t p, size_t q>
  RationalFunction<T, std::max(n + q - 1, m + p - 1), m + q - 1> operator+(
      const RationalFunction<T, p, q>& other) const {
    return {numerator * other.denominator + denominator * other.numerator,
            denominator * other.denominator};
  }

  template <size_t p>
  RationalFunction<T, std::max(n, p* m), m> operator+(
      const Polynomial<T, p>& poly) const {
    return {numerator + denominator * poly, denominator};
  }

  RationalFunction<T, std::max(n, m), m> operator+(const T& scalar) const {
    return {numerator + scalar * denominator, denominator};
  }

  RationalFunction<T, n, m> operator+() const {
    RationalFunction<T, n, m> result = (*this);
    return result;
  }

  template <size_t p, size_t q>
  RationalFunction<T, std::max(n + q - 1, m + p - 1), m + q - 1> operator-(
      const RationalFunction<T, p, q>& other) const {
    return {numerator * other.denominator - denominator * other.numerator,
            denominator * other.denominator};
  }

  template <size_t p>
  RationalFunction<T, std::max(n, p* m), m> operator-(
      const Polynomial<T, p>& poly) const {
    return {numerator - poly * denominator, denominator};
  }

  RationalFunction<T, n, m> operator-(const T& scalar) const {
    return {numerator - scalar * denominator, denominator};
  }

  RationalFunction<T, n, m> operator-() const {
    return {-numerator, denominator};
  }

  RationalFunction<T, m, n> inv() const { return {denominator, numerator}; }
};

template <typename T, size_t n, size_t m, size_t p>
RationalFunction<T, n + p - 1, m> operator*(
    const Polynomial<T, p>& poly, const RationalFunction<T, n, m>& rf) {
  return rf * poly;
}

template <typename T, size_t n, size_t m>
RationalFunction<T, n, m> operator*(const T& scalar,
                                    const RationalFunction<T, n, m>& rf) {
  return rf * scalar;
}

template <typename T, size_t n, size_t m, size_t p>
RationalFunction<T, n + p - 1, m> operator/(
    const Polynomial<T, p>& poly, const RationalFunction<T, n, m>& rf) {
  return rf.inv() * poly;
}

template <typename T, size_t n, size_t m, size_t p>
RationalFunction<T, std::max(n, p* m), m> operator+(
    const Polynomial<T, p>& poly, const RationalFunction<T, n, m>& rf) {
  return rf + poly;
}

template <typename T, size_t n, size_t m>
RationalFunction<T, std::max(n, m), m> operator+(
    const T& scalar, const RationalFunction<T, n, m>& rf) {
  return rf + scalar;
}

template <typename T, size_t n, size_t m, size_t p>
RationalFunction<T, std::max(n, p* m), m> operator-(
    const Polynomial<T, p>& poly, const RationalFunction<T, n, m>& rf) {
  return -rf + poly;
}

template <typename T, size_t n, size_t m>
RationalFunction<T, std::max(n, m), m> operator-(
    const T& scalar, const RationalFunction<T, n, m>& rf) {
  return -rf + scalar;
}
