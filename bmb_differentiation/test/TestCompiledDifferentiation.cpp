#include <bmb_differentiation/compiled/CConstant.h>
#include <bmb_differentiation/compiled/CDeepSimplify.h>
#include <bmb_differentiation/compiled/CDerivative.h>
#include <bmb_differentiation/compiled/CHigherDerivative.h>
#include <bmb_differentiation/compiled/CVariable.h>
#include <gtest/gtest.h>
#include <array>
#include <string>
#include <type_traits>

#ifdef __GNUG__
#include <cxxabi.h>
#include <memory>
#endif

template <typename T>
std::string get_class_name(const T& t) {
  const char* name = typeid(t).name();
#ifdef __GNUG__
  int status;
  std::unique_ptr<char, void (*)(void*)> res{
      abi::__cxa_demangle(name, NULL, NULL, &status), std::free};
  return (status == 0) ? res.get() : name;
#else
  return name;
#endif
}

#define ASSERT_EQUAL(T1, T2) \
  static_assert(std::is_same_v<decltype(T1), decltype(T2)>)

template <typename T>
auto func(T x) {
  using namespace compiled;

  auto y = sin(3_c * x + 4_c);
  return 3_c * y * y;
}

TEST(TestCompiledDifferentiation, testCompiledDifferentiation) {
  using namespace compiled;

  auto x = 0_v;
  auto y = 1_v;

  static_assert(is_constant_v<decltype(3_c)>);
  static_assert(is_constant_v<Zero>);
  static_assert(is_constant_v<One>);
  static_assert(!is_constant_v<decltype(x)>);
  static_assert(is_constant_v<decltype(3_c + 4_c)>);

  ASSERT_EQUAL(0_c, 0_c);
  ASSERT_EQUAL(7_c, 3_c + 4_c);
  ASSERT_EQUAL(4.7_c, 2.1_c + 2.6_c);
  ASSERT_EQUAL(3_c, 3_c + 0_c);
  ASSERT_EQUAL(9_c, pow(3_c, 2_c));
  ASSERT_EQUAL(1_c, pow(3_c, 0_c));

  ASSERT_EQUAL(123456.23_c, 123'456.23_c);
  ASSERT_EQUAL(12'34'56.2'3_c, 123'45'6.23_c);
  ASSERT_EQUAL(65'43'21_v, 654'321_v);

  ASSERT_EQUAL(123e3_c, 123'000_c);
  ASSERT_EQUAL(123e-4_c, 0.0123_c);
  ASSERT_EQUAL(123e0_c, 123_c);
  ASSERT_EQUAL(123e01_c, 1230_c);

  static_assert(getNodeCount(3_c + 0_c) == 1);
  static_assert(getNodeCount(3_c + 2_c) == 1);
  static_assert(getNodeCount(x + y) == 3);
  ASSERT_EQUAL(one, getDerivative(x, x + y));
  ASSERT_EQUAL(x, getHigherDerivative<0>(x, x));
  ASSERT_EQUAL(one, getDerivative(x, x));
  ASSERT_EQUAL(one, getHigherDerivative<1>(x, x));
  static_assert(getNodeCount(getDerivative(x, x + y)) == 1);
  ASSERT_EQUAL(x * 2_c, getDerivative(x, pow(x, 2_c)));
  ASSERT_EQUAL(2_c * x - 4_c, x - 4_c + x);
  ASSERT_EQUAL(x * 6_c, getHigherDerivative<2>(x, pow(x, 3_c)));
  ASSERT_EQUAL(x * 24_c, getHigherDerivative<3>(x, pow(x, 4_c)));
  ASSERT_EQUAL(x * 120_c, getHigherDerivative<4>(x, pow(x, 5_c)));
  auto tt = getHigherDerivative<5>(x, pow(x, 6_c));
  std::cout << get_class_name(tt) << '\n';
  //  ASSERT_EQUAL(x * 720_c, getHigherDerivative<5>(x, pow(x, 6_c)));

  // test func
  using func_type = decltype(getDerivative(0_v, func(0_v)));
  double value = func_type::apply(std::array<double, 1>{3.0});
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
