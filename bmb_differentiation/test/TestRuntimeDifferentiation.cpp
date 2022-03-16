#include <bmb_differentiation/runtime/Expression.h>
#include <bmb_differentiation/runtime/Variable.h>
#include <bmb_math/Matrix.h>
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector.h>
#include <bmb_math/Vector3.h>
#include <gtest/gtest.h>
#include <string>
#include <unordered_map>

TEST(TestRuntimeDifferentiation, testRuntimeDifferentiation) {
  ExprPtr x = Variable::make("x");
  ExprPtr test = 2 * (2 * x + 1);
  ASSERT_EQ(test->nodeCount(), 7);
  ASSERT_EQ(test->simplify()->nodeCount(), 5);

  Quaternion<ExprPtr> quat{Variable::make("q0"), Variable::make("q1"),
                           Variable::make("q2"), Variable::make("q3")};

  Vector3<ExprPtr> w{Variable::make("wx"), Variable::make("wy"),
                     Variable::make("wz")};
  ExprPtr dt = Variable::make("dt");
  Vector<ExprPtr, 4> mat =
      quat.E().transpose() * quat.cong().toDCM() * w * (dt / 2);
  Matrix<ExprPtr, 4, 4> jac;
  for (size_t i = 0; i < 4; i++)
    for (size_t j = 0; j < 4; j++) {
      jac[i][j] = mat[i]->diff("q" + std::to_string(j));
      // std::cout << jac[i][j]->toStr() << '\n';
    }

  Vector<ExprPtr, 3> expr{Variable::make("x"), Variable::make("y"),
                          Variable::make("z")};
  const std::unordered_map<std::string, double> subs{
      {"x", 0}, {"y", 1}, {"z", 2}};
  Vector<double, 3> expr_sub = expr.applyFunc<double>(
      [&subs](const ExprPtr& e) { return e->evaluate(subs); });
  for (size_t i = 0; i < 3; i++) ASSERT_EQ(expr_sub[i], i);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
