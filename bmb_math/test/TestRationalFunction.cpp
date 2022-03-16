#include <bmb_math/RationalFunction.h>
#include <gtest/gtest.h>
#include <cstddef>

TEST(TestRationalFunction, testRationalFunction) {
  static constexpr RationalFunction<double, 1, 2> f{2, 3, 1};
  static constexpr RationalFunction<double, 1, 3> g{4, 5, 0, 1};

  static constexpr auto multiply = f * g;
  static constexpr RationalFunction<double, 1, 4> truth{8, 15, 5, 3, 1};
  static_assert(multiply == truth);

  static constexpr auto divide = f / g;
  static constexpr RationalFunction<double, 3, 2> truth1 = {10, 0, 2, 12, 4};
  static_assert(divide == truth1);

  static constexpr auto add = f + g;
  static constexpr RationalFunction<double, 3, 4> truth2{22, 4, 2, 15, 5, 3, 1};
  static_assert(add == truth2);

  static constexpr auto subtract = f - g;
  static constexpr RationalFunction<double, 3, 4> truth3 = {-2, -4, 2, 15,
                                                            5,  3,  1};
  static_assert(subtract == truth3);

  static constexpr auto g_of_f = g._of_(f);
  static constexpr RationalFunction<double, 3, 3> truth4 = {36, 24, 4,
                                                            49, 30, 5};
  static_assert(g_of_f == truth4);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
