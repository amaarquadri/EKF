#pragma once

#include <bmb_differentiation/compiled/CRatioPower.h>
#include <array>
#include <ratio>
#include <type_traits>

namespace compiled {

template <typename R>
struct Constant {
  using type = Constant<R>;
  using ratio = R;
  static constexpr double value =
      static_cast<double>(R::num) / static_cast<double>(R::den);

  template <size_t n>
  static constexpr double apply(
      const std::array<double, n>& /** variables **/) {
    return value;
  }
};

using Zero = Constant<std::ratio<0>>;
using One = Constant<std::ratio<1>>;
using MinusOne = Constant<std::ratio<-1>>;

Zero zero{};
One one{};
MinusOne minus_one{};

/**
 * Returns true if T is an instance of Constant and false otherwise.
 */
template <typename T>
struct is_constant : std::false_type {};

template <typename T>
struct is_constant<Constant<T>> : std::true_type {};

template <typename T>
inline constexpr bool is_constant_v = is_constant<T>::value;

/**
 * Returns the R in Constant<R>.
 * If the given type is not an instance of Constant, then void is returned.
 */
template <typename T>
struct get_ratio {
  using type = void;
};

template <typename T>
struct get_ratio<Constant<T>> {
  using type = T;
};

template <typename T>
using get_ratio_t = typename get_ratio<T>::type;

/**
 * Parses the given char sequence into an std::ratio.
 * The char sequence can be any valid floating point number.
 */
template <char... cs>
struct parse_ratio;

template <char... cs>
using parse_ratio_t = typename parse_ratio<cs...>::type;

template <>
struct parse_ratio<'.'> {
  using power = std::ratio<1, 10>;
  using type = std::ratio<0>;
};

template <char c>
struct parse_ratio<c> {
  static_assert('0' <= c && c <= '9');

  using power = std::ratio<1>;
  using type = std::ratio<c - '0'>;
};

template <char... cs>
struct parse_ratio<'.', cs...> {
  using power = std::ratio<1, 10>;
  using type = std::ratio_divide<
      parse_ratio_t<cs...>,
      std::ratio_multiply<std::ratio<10>, typename parse_ratio<cs...>::power>>;
};

template <char... cs>
struct parse_ratio<'\'', cs...> : parse_ratio<cs...> {};

template <char... cs>
struct parse_ratio<'-', cs...> {
  // no need to define power in this case, since it should never be used
  // this is because the preceding character will always be an 'e' or 'E'
  using type = std::ratio_multiply<std::ratio<-1>, parse_ratio_t<cs...>>;
};

template <char... cs>
struct parse_ratio<'e', cs...> {
  using power =
      ratio_power_t<std::ratio<10>,
                    std::ratio_subtract<parse_ratio_t<cs...>, std::ratio<1>>>;
  using type = std::ratio<0>;
};

template <char... cs>
struct parse_ratio<'E', cs...> : parse_ratio<'e', cs...> {};

template <char c, char... cs>
struct parse_ratio<c, cs...> {
  static_assert('0' <= c && c <= '9');

  using power =
      std::ratio_multiply<std::ratio<10>, typename parse_ratio<cs...>::power>;
  using type = std::ratio_add<std::ratio_multiply<std::ratio<c - '0'>, power>,
                              parse_ratio_t<cs...>>;
};

template <char... cs>
constexpr Constant<parse_ratio_t<cs...>> operator"" _c() {
  return {};
}

}  // namespace compiled
