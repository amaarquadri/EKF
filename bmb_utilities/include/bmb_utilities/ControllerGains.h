#pragma once

struct ControllerGains {
  // proportional, integral, derivative, feed forward, derivative low pass
  // filter coefficient
  double kp, ki, kd, kff, N;

  constexpr ControllerGains(const double& kp = 0, const double& ki = 0,
                            const double& kd = 0, const double& kff = 0,
                            const double& N = 100)
      : kp(kp), ki(ki), kd(kd), kff(kff), N(N) {}
};
