#include "TestRatioPower.h"
#include "CRatioPower.h"

#include <iostream>
#include <ratio>

void testRatioPower() {
  using namespace compiled;

  static_assert(!can_ratio_power_v<std::ratio<0>, std::ratio<-5>>);
  static_assert(!can_ratio_power_v<std::ratio<3, 4>, std::ratio<1, 2>>);

  static_assert(std::is_same_v<std::ratio<9>, ratio_power_t<std::ratio<3>, std::ratio<2>>>);
  static_assert(std::is_same_v<std::ratio<1024>, ratio_power_t<std::ratio<2>, std::ratio<10>>>);
  static_assert(std::is_same_v<std::ratio<1>, ratio_power_t<std::ratio<5>, std::ratio<0>>>);
  static_assert(std::is_same_v<std::ratio<0>, ratio_power_t<std::ratio<0>, std::ratio<10>>>);

  std::cout << "Passed All Tests for Ratio Power!\n";
}
