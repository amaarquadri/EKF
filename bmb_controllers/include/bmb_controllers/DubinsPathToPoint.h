#pragma once

#include <bmb_controllers/DubinsCurve.h>
#include <bmb_controllers/PosVelState.h>
#include <bmb_math/Utility.h>
#include <bmb_math/Vector.h>
#include <cmath>

template<typename T>
class DubinsPathToPoint {
  using Path = std::array<DubinsCurve<T>, 2>;
  using iterator = typename Path::iterator;
  using const_iterator = typename Path::const_iterator;

  /**
   * This constructor will create this DubinsPathToPoint with unspecified data
   * inside of it. It is the caller's responsibility to handle this.
   */
  DubinsPathToPoint() = default;

  explicit DubinsPathToPoint(const Path& path) : path(path) {}

  DubinsPathToPoint(const DubinsPathToPoint<T>& other) { (*this) = other; }

  DubinsPathToPoint& operator=(const DubinsPathToPoint<T>& other) {
    for (size_t i = 0; i < path.size(); i++) path[i] = other[i];
    return *this;
  }

  static DubinsPathToPoint create(const PosVelState<T>& start,
                                  const Vector<T, 2>& goal, const T& radius) {
    // TODO: implement
    // Based on:
    // https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.823.5493&rep=rep1&type=pdf
    const T radius_squared = radius * radius;
    const Vector2 right_dir = bmb_math::ROT_90_CW * start.vel;
    if (right_dir.dot(goal - start.pos) > 0) {
      // goal is on the right
      const Vector2 center =
          start.pos + radius * right_dir / right_dir.magnitude();
      const Vector2 d1_vec = goal - center;
      const T d1_squared = d1_vec.magnitudeSquared();
      if (d1_squared < radius_squared) {
        // LR
      } else {
        // RS
        const T start_angle = bmb_math::atan2(-right_dir);
        const T dist = std::sqrt(d1_squared - radius_squared);
        const T delta_angle = bmb_math::normalizeAngle(
            start_angle - std::atan2(dist, radius) - bmb_math::atan2(d1_vec));

        const DubinsCurve<T> c1{center, radius, start_angle, delta_angle};
        const DubinsCurve<T> c2{
            center + bmb_math::polarToVec(radius, start_angle), goal};
        return DubinsPath{{c1, c2}};
      }
    } else {
      // goal is on the left
      const Vector2 center =
          start.pos - right_dir * radius / right_dir.magnitude();
      const Vector2 d1_vec = goal - center;
      const T d1_squared = d1_vec.magnitudeSquared();
      if (d1_squared < radius_squared) {
        // RL
      } else {
        // LS
        const T start_angle = bmb_math::atan2(right_dir);
        const T dist = std::sqrt(d1_squared - radius_squared);
        const T delta_angle = bmb_math::normalizeAngle(
            start_angle - std::atan2(dist, radius) - bmb_math::atan2(d1_vec));

        const DubinsCurve<T> c1{center, radius, start_angle, delta_angle};
        const DubinsCurve<T> c2{
            center + bmb_math::polarToVec(radius, start_angle), goal};
        return DubinsPath{{c1, c2}};
      }
    }
  }

  const DubinsCurve<T>& operator[](const size_t& index) const {
    return path[index];
  }

  DubinsCurve<T>& operator[](const size_t& index) { return path[index]; }

  T getLength() const { return path[0].getLength() + path[1].getLength(); }

  bool operator<(const DubinsPathToPoint<T>& other) const {
    return getLength() < other.getLength();
  }

  Vector<T, 2> closestPoint(const Vector<T, 2>& p) const {
    return std::min(path[0].closestPoint(p), path[1].closestPoint(p),
                    [&p](const Vector2& first, const Vector2& second) {
                      return (first - p).magnitudeSquared() <
                             (second - p).magnitudeSquared();
                    });
  }

  [[nodiscard]] std::string toStr() const {
    std::stringstream out;
    out << path[0].toStr() << ", " << path[1].toStr();
    return out.str();
  }

  template <typename OStream>
  void toCSV(OStream& out) const {
    for (size_t i = 0; i < path.size(); i++) path[i].toCSV(out);
  }

  iterator begin() { return path.begin(); }

  iterator end() { return path.end(); }

  const_iterator begin() const { return path.begin(); }

  const_iterator end() const { return path.end(); }

  const_iterator cbegin() const { return path.cbegin(); }

  const_iterator cend() const { return path.cend(); }

 private:
  using Vector2 = Vector<T, 2>;

  Path path;
};
