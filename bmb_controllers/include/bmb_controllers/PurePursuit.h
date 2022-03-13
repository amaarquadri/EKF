#pragma once

#include <bmb_controllers/PosVelState.h>
#include <bmb_controllers/DubinsPath.h>
#include <bmb_math/Utility.h>
#include <cmath>
#include <utility>

template <typename T>
class PurePursuit {
 public:
  /**
   * This constructor will create this PurePursuit with unspecified data inside
   * of it. It is the caller's responsibility to handle this.
   */
  PurePursuit() = default;

  PurePursuit(const T& lookahead_time, const T& min_radius,
              const DubinsPath<T>& path)
      : lookahead_time(lookahead_time), min_radius(min_radius), path(path) {}

  void updatePath(const DubinsPath<T>& new_path) { this->path = new_path; }

  /**
   * A positive returned angular velocity indicates a turn to the left
   *
   * @return An std::pair of a bool indicating whether a re-plan is needed, and
   * the commanded angular velocity
   */
  std::pair<bool, T> pursue(const PosVelState<T>& state) const {
    // TODO: can this be a stateless free global function?

    // Pursuit algorithm roughly based off of this article:
    // https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
    const Vector2 ideal_target = state.pos + state.vel * lookahead_time;
    Vector2 target = path.closestPoint(ideal_target);
    // TODO: add check to see if we are too far off course

    // readjust to a coordinate system where we are at the origin and moving in
    // the direction of the x-axis
    target -= state.pos;
    target = bmb_math::getRotationMatrix(state.vel).transpose() * target;

    if (target[0] <= 0) {
      // target is behind us (or to the side of us)
      return {true, 0};
    }

    // now the center of rotation must be at the point of intersection of the
    // y-axis and the perpendicular bisector of the line segment connecting
    // target to the origin simple geometry shows that the y coordinate where
    // this occurs is given by the following note that this will be negative if
    // target[1] < 0, as it should be in that case to indicate a right turn
    const T radius = target.magnitudeSquared() / (2 * target[1]);

    if (std::fabs(radius) < min_radius) {
      // required turn is too sharp, request re-plan
      return {true, 0};
    }

    return {false, state.vel.magnitude() / radius};
  }

  /**
   * Outputs the anticipated trajectory that will be followed based on the
   * current applied settings and path. The state will be integrated forwards in
   * time with a step size of STEP_SIZE in time. Integration will be stopped
   * once a replan is requested.
   */
  template <typename OStream>
  void toCSV(OStream& out, const PosVelState<T>& start) const {
    PosVelState<T> state = start;
    out << state.pos[0] << ", " << state.pos[1] << std::endl;

    while (true) {
      auto [should_replan, ang_vel] = pursue(state);
      if (should_replan) break;

      state.pos += state.vel * STEP_SIZE;
      state.vel = bmb_math::getRotationMatrix(ang_vel * STEP_SIZE) * state.vel;
      out << state.pos[0] << ", " << state.pos[1] << std::endl;
    }
  }

 private:
  using Vector2 = Vector<T, 2>;
  static constexpr T STEP_SIZE = static_cast<T>(0.01);
  T lookahead_time;
  T min_radius;
  DubinsPath<T> path;
};