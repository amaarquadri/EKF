cmake_minimum_required(VERSION 3.17)
project(FYDP)

set(CMAKE_CXX_STANDARD 20)

include_directories(./include)
include_directories(./include/differentiation)
include_directories(./include/numc)
include_directories(./include/kalman_filter)
include_directories(./include/utils)

add_executable(FYDP tests.cpp
        src/utils/math_utils.cpp
        src/kalman_filter/SensorFilter.cpp
        src/kalman_filter/BasicSensorFilter.cpp
        src/kalman_filter/KF.cpp
        src/kalman_filter/UKF.cpp
        src/kalman_filter/EKF.cpp
        src/AircraftState.cpp
        src/ControlInputs.cpp
        src/AppliedLoads.cpp
        src/SensorModels.cpp
        src/differentiation/Expression.cpp
        src/differentiation/Constant.cpp
        src/differentiation/Zero.cpp
        src/differentiation/One.cpp
        src/differentiation/Nan.cpp
        src/differentiation/Variable.cpp
        src/differentiation/BinaryOperator.cpp
        src/differentiation/UnaryOperator.cpp
        src/differentiation/Sum.cpp
        src/differentiation/Difference.cpp
        src/differentiation/Product.cpp
        src/differentiation/Quotient.cpp
        src/differentiation/Power.cpp
        src/differentiation/Exp.cpp
        src/differentiation/Log.cpp
        src/differentiation/Sin.cpp
        src/differentiation/Cos.cpp
        src/numc/Wrench.cpp
        src/numc/Primes.cpp
        test/TestVector.cpp
        test/TestVector3.cpp
        test/TestQuaternion.cpp
        test/TestMatrix.cpp
        test/TestDifferentiation.cpp
        test/TestPrimes.cpp
        test/TestFourier.cpp
        src/PID.cpp)
