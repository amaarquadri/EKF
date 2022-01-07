#pragma once

#include <type_traits>

namespace compiled {

template <typename L, typename R>
struct difference {
  using type = difference<L, R>;
  using left_operand_type = L;
  using right_operand_type = R;

  static inline double apply(const double* variables) {
    return L::apply(variables) - R::apply(variables);
  }
};

template <typename L, typename R>
using difference_t = typename difference<L, R>::type;

template<typename T>
struct is_difference : std::false_type {};

template<typename L, typename R>
struct is_difference<difference<L, R>> : std::true_type {};

template<typename T>
inline constexpr bool is_difference_v = is_difference<T>::value;

}  // namespace compiled