#include <bmb_math/Vector.h>
#include <gtest/gtest.h>

static int roundDown(const double& num) { return (int)num; }

TEST(TestVector, testVector) {
  Vector<double, 5> x;
  for (size_t i = 0; i < 5; i++) ASSERT_EQ(x[i], 0);
  Vector<double, 5> y;
  x[0] = 2;
  y[0] = 4;
  x[1] = -4;
  y[1] = 7;

  Vector<double, 5> z = x + y;
  ASSERT_EQ(z[0], 6);
  ASSERT_EQ(z[1], 3);
  y[2] = 8;
  ASSERT_EQ(z[2], 0);
  z += y;
  ASSERT_EQ(z[2], 8);
  ASSERT_EQ(z[0], 10);

  Vector<double, 5> a{1, 2, 3, 4, 5};
  Vector<double, 2> b{1, 2};
  a += b;
  ASSERT_EQ(a[0], 2);
  ASSERT_EQ(a[1], 4);
  ASSERT_EQ(a[3], 4);
  a.operator+=<1, 2>(b);
  ASSERT_EQ(a[0], 2);
  ASSERT_EQ(a[1], 5);
  ASSERT_EQ(a[3], 6);

  Vector<double, 5> test{1.1, -4.5, 3, 4, 0};
  Vector<int, 5> test_floor = test.applyFunc<int>(&roundDown);
  for (size_t i = 0; i < 5; i++) ASSERT_EQ(test_floor[i], (int)test[i]);

  Vector<int, 5> test_sq_int =
      test.applyFunc<int>([](const double& d) { return (int)(d * d); });
  for (size_t i = 0; i < 5; i++)
    ASSERT_EQ(test_sq_int[i], (int)(test[i] * test[i]));
}

TEST(TestVector, testConstexpr) {
  static constexpr Vector<double, 5> data{0, 1, 2, 3, 4};
  static constexpr Vector<double, 5> other = data * 10 + data * data;
  static_assert(other[2] == 2 * 10 + 2 * 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
