#include "TestQuaternion.h"
#include "Quaternion.h"
#include <cassert>
#include <iostream>

void testQuaternion() {
    Quaternion<> first = Quaternion<>::identity();
    Quaternion<> second{};
    second = first;
    second.q1 = 1;
    assert(first.q1 == 0);
    assert(second.q0 == 1);
    assert(second.q1 == 1);

    std::cout << "Passed All Tests for Quaternion!" << std::endl;
}
