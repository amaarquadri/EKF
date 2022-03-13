#include "bmb_state_estimation/KF.h"
#include <bmb_math/MessageUtilities.h>
#include <bmb_math/Quaternion.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_world_model/AppliedLoads.h>
#include <bmb_world_model/Constants.h>
#include <bmb_world_model/SensorModels.h>
#include <bmb_world_model/WrenchUtilities.h>

KF::KF() {
  x[q0] = 1;
  P[px][px] = P[py][py] = 0;
}

void KF::update(const bmb_msgs::SensorMeasurements& sensor_measurements,
                const bmb_msgs::ControlInputs& control_inputs,
                const double& dt) {
  // calculate current loads once and store in an instance variable so that it
  // can be used throughout
  current_loads = getAppliedLoads(getOutput(), control_inputs);
  current_accel = bmb_world_model::toAccel(current_loads);

  // delegate to subclasses
  updateKF(sensor_measurements, control_inputs, dt);
}

bmb_msgs::AircraftState KF::getOutput() const {
  return bmb_math::as_msg(x.slice<0, bmb_msgs::AircraftState::SIZE>());
}

Vector<double, n> KF::f(const Vector<double, n>& state,
                        const double& dt) const {
  const Quaternion<double> quat{state[q0], state[q1], state[q2], state[q3]};

  Vector<double, n> state_new = state;

  const Vector3<double> v{state[vx], state[vy], state[vz]};
  const Vector3<double> v_abs = quat.unrotate(v);

  const Vector3<double> w{state[wx], state[wy], state[wz]};
  const Vector3<double> w_abs = quat.unrotate(w);

  Quaternion<double> quat_new = quat + quat.E().transpose() * w_abs * (dt / 2);
  quat_new.normalize();

  const Vector3<double> mag{state[magx], state[magy], state[magz]};
  const Vector3<double> mag_b{state[mag_bx], state[mag_by], state[mag_bz]};
  const Vector3<double> mag_new =
      quat_new.rotate(quat.unrotate(mag - mag_b)) + mag_b;

  for (size_t i = 0; i < 3; i++) {
    state_new[px + i] += v_abs[i] * dt;
    state_new[q0 + i] = quat_new[i];
    state_new[vx + i] += current_accel.linear[i] * dt;
    state_new[wx + i] += current_accel.angular[i] * dt;
    state_new[magx + i] = mag_new[i];
  }
  state_new[q3] = quat_new.q3;  // not included in for loop

  return state_new;
}

Vector<double, p> KF::h(const Vector<double, n>& state,
                        const double& /** dt **/) {
  return bmb_math::as_vector(getSensorMeasurements(
      getOutput(), state.slice<accel_bx, accel_bz + 1>(),
      state.slice<gyro_bx, gyro_bz + 1>(), current_accel));
}
