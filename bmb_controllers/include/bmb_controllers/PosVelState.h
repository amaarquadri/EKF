#pragma once

#include <bmb_math/Vector.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>

template<typename T>
struct PosVelState {
  Vector<T, 2> pos;
  Vector<T, 2> vel;

  PosVelState() = default;

  PosVelState(const Vector<T, 2>& pos, const Vector<T, 2>& vel)
      : pos(pos), vel(vel) {}

  PosVelState(const bmb_msgs::AircraftState& msg) {
    pos[0] = msg.pose.position.x;
    pos[1] = msg.pose.position.y;
    vel[0] = msg.twist.linear.x;
    vel[1] = msg.twist.linear.y;
  }

  PosVelState(const bmb_msgs::ReferenceCommand& msg) {
    pos[0] = msg.x_pos;
    pos[1] = msg.y_pos;
    vel[0] = msg.x_vel;
    vel[1] = msg.y_vel;
  }

  PosVelState& operator=(const PosVelState<T>& other) {
    pos = other.pos;
    vel = other.vel;
    return (*this);
  }

  PosVelState(const PosVelState<T>& other) { (*this) = other; }
};
