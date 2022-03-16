#pragma once

#include <bmb_math/RationalFunction.h>
#include <bmb_utilities/MathUtils.h>
#include <array>
#include <cassert>
#include <initializer_list>
#include <iostream>

template <typename T, size_t n, size_t m>
class TransferFunction : public RationalFunction<T, n, m> {
  static_assert(m >= n, "Transfer function is not proper!");
  static_assert(n != 0 && m != 0,
                "Numerator and denominator must have at least 1 element!");

  Vector<T, m>
      past_inputs;  // only use the n oldest of these values at each time step
  Vector<T, m - 1> past_outputs;

 public:
  TransferFunction() = default;

  TransferFunction(std::initializer_list<T> elements)
      : RationalFunction<T, n, m>(elements) {}

  TransferFunction(const Polynomial<T, n>& numerator,
                   const Polynomial<T, m>& denominator)
      : RationalFunction<T, n, m>(numerator, denominator) {}

  static TransferFunction<T, std::max(n, m), std::max(n, m)>
  getDiscreteFromContinuousCoefficients(const Polynomial<T, n>& num,
                                        const Polynomial<T, m>& den,
                                        const T& dt) {
    return TransferFunction<T, n, m>{num, den}.discretize(dt);
  }

  // allow implicit conversions
  TransferFunction(const RationalFunction<T, n, m>& rf)
      : TransferFunction(rf.numerator, rf.denominator) {
  }  // NOLINT(google-explicit-constructor)

  TransferFunction& operator=(const TransferFunction<T, n, m>& other) {
    this->numerator = other.numerator;
    this->denominator = other.denominator;
    this->past_inputs = other.past_inputs;
    this->past_outputs = other.past_outputs;
    return *this;
  }

  TransferFunction(const TransferFunction<T, n, m>& other) { (*this) = other; }

  TransferFunction<T, n, std::max(n, m)> feedbackLoop() const {
    return {this->numerator, this->numerator + this->denominator};
  }

  template <size_t p, size_t q>
  TransferFunction<T, n + p - 1, std::max(n + p - 1, m + q - 1)> feedbackLoop(
      const TransferFunction<T, p, q>& other) const {
    TransferFunction<T, n + p - 1, m + q - 1> CP = (*this) * other;
    return {CP.numerator, CP.numerator + CP.denominator};
  }

  TransferFunction<T, n + bmb_utilities::heaviside_difference(m, n),
                   m + bmb_utilities::heaviside_difference(n, m)>
  discretize(const T& dt = 1E-4) const {
    // use trapezoidal method since SCH looks hard
    static constexpr size_t p = n + bmb_utilities::heaviside_difference(m, n);
    static constexpr size_t q = m + bmb_utilities::heaviside_difference(n, m);
    RationalFunction<T, 2, 2> trapezoidal{Polynomial<T, 2>{-2, 2},
                                          Polynomial<T, 2>{dt, dt}};

    TransferFunction<T, p, q> discretized = this->_of_(trapezoidal);
    discretized.normalize();
    return discretized;
  }

  /**
   * The output should not be used in algebraic loops if n == m
   */
  T next_output(const T& input) {
    T next_output = static_cast<T>(0);
    past_inputs = past_inputs.pushFrontPopBack(input);
    for (size_t i = 0; i < m - 1; i++)
      next_output -= this->denominator[i] * past_outputs[m - 2 - i];
    for (size_t i = 0; i < std::min(n, m); i++)
      next_output += this->numerator[i] * past_inputs[m - 1 - i];
    next_output /= this->denominator[m - 1];
    past_outputs = past_outputs.pushFrontPopBack(next_output);
    return next_output;
  }

  template <size_t output_size>
  Vector<T, output_size> step() {
    Vector<T, output_size> step_response;
    for (size_t i = 0; i < output_size; i++)
      step_response[i] = next_output(static_cast<T>(1));
    return step_response;
  }
};

template <typename T, size_t n, size_t m>
TransferFunction<T, n + bmb_utilities::heaviside_difference(m, n),
                        m + bmb_utilities::heaviside_difference(n, m)>
c2d(const TransferFunction<T, n, m>& tf, const T& dt = 1e-4) {
  TransferFunction<T, n + bmb_utilities::heaviside_difference(m, n),
                   m + bmb_utilities::heaviside_difference(n, m)>
      discrete = tf.discretize(dt);
  return discrete;
}
