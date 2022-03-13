#pragma once

#include <bmb_differentiation/compiled/CConstant.h>
#include <array>
#include <cstddef>
#include <limits>

namespace compiled {

template <size_t id>
struct Variable {
  using type = Variable<id>;
  static constexpr size_t id_value = id;

  template <size_t n>
  static inline double apply(const std::array<double, n>& variables) {
    static_assert(id < n, "Input number of variables is not large enough!");
    return variables[id];
  }
};

/**
 * Returns true if T is an instance of Variable and false otherwise.
 */
template <typename T>
struct is_variable : std::false_type {};

template <size_t id>
struct is_variable<Variable<id>> : std::true_type {};

template <typename T>
inline constexpr bool is_variable_v = is_variable<T>::value;

/**
 * Parses the given char sequence into a size_t.
 * The char sequence can be any valid natural number.
 */
template <char... cs>
struct parse_size_t {
 private:
  using R = parse_ratio_t<cs...>;
  static_assert(R::den == 1 && R::num >= 0,
                "literal does not evaluate to a natural number");
  static_assert(R::num <= std::numeric_limits<size_t>::max(),
                "literal does not fit in a size_t");

 public:
  static constexpr size_t value = static_cast<size_t>(R::num);
};

template <char... cs>
inline constexpr size_t parse_size_t_v = parse_size_t<cs...>::value;

template <char... cs>
constexpr Variable<parse_size_t_v<cs...>> operator"" _v() {
  return {};
}

}  // namespace compiled
