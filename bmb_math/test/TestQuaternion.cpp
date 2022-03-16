#include <bmb_math/Quaternion.h>
#include <gtest/gtest.h>

TEST(TestQuaternion, testQuaternion) {
  // TODO: figure out how to make this static constexpr
  static const Quaternion<double> first = Quaternion<double>::identity();
  Quaternion<double> second{};
  second = first;
  second.q1 = 1;
  ASSERT_EQ(first.q1, 0);
  ASSERT_EQ(second.q0, 1);
  ASSERT_EQ(second.q1, 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
