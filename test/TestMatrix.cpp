#include "TestMatrix.h"
#include "Matrix.h"
#include <string>
#include <cassert>
#include <iostream>

void testMatrix() {
    Matrix<2, 3, uint8_t> mat{};
    for (size_t i = 0; i < 2; i++) for (size_t j = 0; j < 3; j++) assert(mat[i][j] == 0);

    Matrix<3, 4, uint8_t> mat2 = Matrix<3, 4, uint8_t>::identity();
    Matrix<2, 4, uint8_t> product = mat * mat2;
    Matrix<2, 4, uint8_t> z = Matrix<2, 4, uint8_t>::zeros();
    for (size_t i = 0; i < 2; i++) for (size_t j = 0; j < 4; j++) assert(product[i][j] == z[i][j]);

    Matrix<1, 2, int> d_mat = Matrix<1, 2, int>::identity();
    Matrix<1, 2, int> s_mat = d_mat.applyFunc<int>([] (const int &num) { return num * num; });
    assert(s_mat[0][0] == 1);
    assert(s_mat[0][1] == 0);

    std::cout << "Passed All Tests for Matrix!" << std::endl;
}
