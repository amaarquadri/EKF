#pragma once

#include <cstdint>
#include <ratio>

namespace compiled {

/**
 * Returns true if R is equivalent to std::ratio<0> and false otherwise.
 */
template <typename R>
struct is_ratio_zero {
  static constexpr bool value = R::num == 0 && R::den == 1;
};

template <typename R>
inline constexpr bool is_ratio_zero_v = is_ratio_zero<R>::value;

/**
 * Returns true if R is equivalent to std::ratio<1> and false otherwise.
 */
template <typename R>
struct is_ratio_one {
  static constexpr bool value = R::num == 1 && R::den == 1;
};

template <typename R>
inline constexpr bool is_ratio_one_v = is_ratio_one<R>::value;

/**
 * Returns true if T1 ^ T2 can be represented as a std::ratio using ratio_power.
 * If either of T1 or T2 are not instances of std::ratio, then false is
 * returned.
 */
template <typename T1, typename T2>
struct can_ratio_power {
  // return false in the default case where T1 and T2 are not instances of
  // std::ratio
  static constexpr bool value = false;
};

template <std::intmax_t R1_num, std::intmax_t R1_den, std::intmax_t R2_num,
          std::intmax_t R2_den>
struct can_ratio_power<std::ratio<R1_num, R1_den>, std::ratio<R2_num, R2_den>> {
 private:
  using R1 = std::ratio<R1_num, R1_den>;
  using R2 = std::ratio<R2_num, R2_den>;

 public:
  // return true if the exponent is an integer
  // the exponent must also be non-negative in the case where the base is zero
  static constexpr bool value =
      R2::den == 1 && !(is_ratio_zero_v<R1> && R2::num < 0);
};

template <typename R1, typename R2>
inline constexpr bool can_ratio_power_v = can_ratio_power<R1, R2>::value;

/**
 * Computes R1 ^ R2 as an std::ratio.
 * This static_asserts that can_ratio_power_v<R1, R2>.
 */
template <typename R1, typename R2>
struct ratio_power;

template <typename R1, typename R2>
using ratio_power_t = typename ratio_power<R1, R2>::type;

template <typename R1, typename R2>
struct ratio_power {
  static_assert(can_ratio_power_v<R1, R2>,
                "Result cannot be computed as an std::ratio!");

 private:
  static constexpr auto impl() {
    // 0 ^ 0 -> 1
    if constexpr (is_ratio_zero_v<R1> && is_ratio_zero_v<R2>)
      return std::ratio<1>{};
    // 0 ^ 1 -> 0
    else if constexpr (is_ratio_zero_v<R1> && is_ratio_one_v<R2>)
      return std::ratio<0>{};
    // 1 ^ 0 -> 1
    else if constexpr (is_ratio_one_v<R1> && is_ratio_zero_v<R2>)
      return std::ratio<1>{};
    // 1 ^ 1 -> 1
    else if constexpr (is_ratio_one_v<R1> && is_ratio_one_v<R2>)
      return std::ratio<1>{};
    // B ^ 0 -> 1
    else if constexpr (is_ratio_zero_v<R2>)
      return std::ratio<1>{};
    // B ^ 1 -> B
    else if constexpr (is_ratio_one_v<R2>)
      return R1{};
    // 0 ^ E -> 0, we don't care about cases where E < 0 due to the static
    // assert
    else if constexpr (is_ratio_zero_v<R1>)
      return std::ratio<0>{};
    // 1 ^ E -> 1
    else if constexpr (is_ratio_one_v<R1>)
      return std::ratio<1>{};
    // for negative exponents, flip the base and recurse
    else if constexpr (R2::num < 0)
      return ratio_power_t<std::ratio<R1::den, R1::num>,
                           std::ratio<-R2::num>>{};
    // recursive case
    else
      return std::ratio_multiply<R1,
                                 ratio_power_t<R1, std::ratio<R2::num - 1>>>{};
  }

 public:
  using type = decltype(impl());
};

}  // namespace compiled
