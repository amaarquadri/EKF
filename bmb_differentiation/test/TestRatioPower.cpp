#include <bmb_differentiation/compiled/CRatioPower.h>
#include <gtest/gtest.h>
#include <ratio>

TEST(TestRatioPower, testRatioPower) {
  using namespace compiled;

  static_assert(!can_ratio_power_v<std::ratio<0>, std::ratio<-5>>);
  static_assert(!can_ratio_power_v<std::ratio<3, 4>, std::ratio<1, 2>>);

  static_assert(std::is_same_v<std::ratio<9>,
                               ratio_power_t<std::ratio<3>, std::ratio<2>>>);
  static_assert(std::is_same_v<std::ratio<1024>,
                               ratio_power_t<std::ratio<2>, std::ratio<10>>>);
  static_assert(std::is_same_v<std::ratio<1>,
                               ratio_power_t<std::ratio<5>, std::ratio<0>>>);
  static_assert(std::is_same_v<std::ratio<0>,
                               ratio_power_t<std::ratio<0>, std::ratio<10>>>);

  static_assert(std::is_same_v<std::ratio<1, 1000>,
                               ratio_power_t<std::ratio<10>, std::ratio<-3>>>);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
