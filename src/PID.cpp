#include "PID.h"
#include "Vector.h"
#include "Vector3.h"
#include <cmath>
#include "AppliedLoads.h"
#include "math_utils.h"

double PID::output_signal(double error, double dt) {
    double output_signal = 0;
    if (K_p) output_signal += K_p * error;
    if (K_I) {
        error_area += error * dt;
        output_signal += K_I * error_area;
    }
    if (K_d) {
        output_signal += K_d * (error - last_error) / dt;
        last_error = error;
    }
    return output_signal;
}

double PID::proportional_signal(double K_p, double error) {
    return K_p * error;
}

static double const heading_controller(Vector<2> const *waypoint_command, Vector<2> const *position) {
    Vector<2> diff = *waypoint_command - *position;
    return atan2(diff[0], diff[1]);
}

static Vector3<> const throttle_controller(Vector3<> const *airspeed_command, Vector3<> const *body_velocity, double dt) {
    Vector3<> error = *airspeed_command - *body_velocity;
    double K_ff = 1;
    double K_p = 1,  K_i = 1, K_d = 1;
    PID pid = PID(K_p, K_i, K_d);
    Vector3<> u;
    for (int i = 0; i < 3; i++) {
       u[i] = pid.output_signal(error[i], dt);
    }
    return K_ff * *airspeed_command + u;
}

static Vector3<> const roll_controller(Vector3<> const *heading_command, Vector3<> const *euler_angles, double dt) {
    Vector3<> error = *heading_command - *euler_angles;
    double K_p = 1,  K_i = 1, K_d = 1;
    PID pid = PID(K_p, K_i, K_d);
    Vector3<> u;
    for (int i = 0; i < 3; i++) {
        u[i] = utils::saturation<double>(pid.output_signal(error[i], dt), 5);
    }
    return u;
}

static Vector3<> propeller_controller(const Vector3<> &throttle_command) {
    double K_p = 1;
    return K_p * throttle_command;
}

static Vector3<> const elevator_controller(Vector3<> const *pitch_command, Vector3<> const *aircraft_pitch, double dt) {
    Vector3<> error = *pitch_command - *aircraft_pitch;
    double K_ff = 1;
    double K_p = 1,  K_i = 1, K_d = 1;
    PID pid = PID(K_p, K_i, K_d);
    Vector3<> u;
    for (int i = 0; i < 3; i++) {
        u[i] = pid.output_signal(error[i], dt);
    }
    return K_ff * *pitch_command + u;
}

static Vector<2, Vector3<>> const aileron_controller(Vector3<> const *roll_command, Vector3<> const *aircraft_roll, double dt) {
    Vector3<> error = *roll_command - *aircraft_roll;
}