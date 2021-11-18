#include "KF.h"
#include "Quaternion.h"
#include "Constants.h"
#include "SensorModels.h"

KF::KF() {
    x[q0] = 1;
    P[px][px] = P[py][py] = 0;
}

void KF::update(const SensorMeasurements<> &sensorMeasurements, const ControlInputs &control_inputs, const double &dt) {
    applied_loads.update(control_inputs);

    // calculate current loads once and store in an instance variable so that it can be used throughout
    current_loads = applied_loads.getAppliedLoads(x);
    current_accel = toAccel(current_loads);

    // delegate to subclasses
    updateKF(sensorMeasurements, dt);
}

AircraftState KF::getOutput() const {
    return AircraftState{Vector3<>{x[px], x[py], x[pz]},
                         Quaternion<>{x[q0], x[q1], x[q2], x[q3]},
                         Vector3<>{x[vx], x[vy], x[vz]},
                         Vector3<>{x[wx], x[wy], x[wz]},
                         current_accel.angular,
                         current_accel.linear};
}

Vector<n> KF::f(const Vector<n> &state, const double &dt) const {
    Quaternion<> quat{state[q0], state[q1], state[q2], state[q3]};

    Vector<n> state_new = state;

    Vector3<> v{state[vx], state[vy], state[vz]};
    Vector3<> v_abs = quat.unrotate(v);

    Vector3<> w{state[wx], state[wy], state[wz]};
    Vector3<> w_abs = quat.unrotate(w);

    Quaternion<> quat_new = quat + quat.E().transpose() * w_abs * (dt / 2);
    quat_new.normalize();

    Vector3<> mag{state[magx], state[magy], state[magz]};
    Vector3<> mag_b{state[mag_bx], state[mag_by], state[mag_bz]};
    Vector3<> mag_new = quat_new.rotate(quat.unrotate(mag - mag_b)) + mag_b;

    for (size_t i = 0; i < 3; i++) {
        state_new[px + i] += v_abs[i] * dt;
        state_new[q0 + i] = quat_new[i];
        state_new[vx + i] += current_accel.linear[i] * dt;
        state_new[wx + i] += current_accel.angular[i] * dt;
        state_new[magx + i] = mag_new[i];
    }
    state_new[q3] = quat_new.q3; // not included in for loop

    return state_new;
}

Vector<p> KF::h(const Vector<n> &state, const double & /** dt **/) {
    return getSensorMeasurements(state, current_accel).getZ();
}
