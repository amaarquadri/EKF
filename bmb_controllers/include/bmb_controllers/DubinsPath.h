#pragma once

#include <bmb_controllers/DubinsCurve.h>
#include <bmb_controllers/PosVelState.h>
#include <bmb_math/Matrix.h>
#include <bmb_math/Vector.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

template <typename T>
class DubinsPath {
  // TODO: look into implementing these:
  //  https://arxiv.org/pdf/1804.07238.pdf
  //  https://ece.uwaterloo.ca/~sl2smith/papers/2016CDC_Efficient_Dubins.pdf
 public:
  using Path = std::array<DubinsCurve<T>, 3>;
  using iterator = typename Path::iterator;
  using const_iterator = typename Path::const_iterator;

  /**
   * This constructor will create this DubinsPath with unspecified data inside
   * of it. It is the caller's responsibility to handle this.
   */
  DubinsPath() = default;

  explicit DubinsPath(const Path& path) : path(path) {}

  DubinsPath(const DubinsPath<T>& other) { (*this) = other; }

  DubinsPath& operator=(const DubinsPath<T>& other) {
    for (size_t i = 0; i < path.size(); i++) path[i] = other[i];
    return *this;
  }

  static DubinsPath create(const PosVelState<T>& start,
                           const PosVelState<T>& goal, const T& radius) {
    // Based on:
    // https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/

    // start with the shorter between the RSR and LSL path, since they are
    // always possible
    DubinsPath best_path =
        std::min(getCSCOuterTangent<true>(start, goal, radius),    // RSR
                 getCSCOuterTangent<false>(start, goal, radius));  // LSL

    // try RSL path
    auto [is_valid, path] = getCSCInnerTangent<true>(start, goal, radius);
    if (is_valid) best_path = std::min(best_path, path);

    // try LSR path
    std::tie(is_valid, path) = getCSCInnerTangent<false>(start, goal, radius);
    if (is_valid) best_path = std::min(best_path, path);

    // try RLR path
    std::tie(is_valid, path) = getCCC<true>(start, goal, radius);
    if (is_valid) best_path = std::min(best_path, path);

    // try LRL path
    std::tie(is_valid, path) = getCCC<false>(start, goal, radius);
    if (is_valid) best_path = std::min(best_path, path);

    return best_path;
  }

  const DubinsCurve<T>& operator[](const size_t& index) const {
    return path[index];
  }

  DubinsCurve<T>& operator[](const size_t& index) { return path[index]; }

  T getLength() const {
    return path[0].getLength() + path[1].getLength() + path[2].getLength();
  }

  bool operator<(const DubinsPath<T>& other) const {
    return getLength() < other.getLength();
  }

  Vector<T, 2> closestPoint(const Vector<T, 2>& p) const {
    return std::min({path[0].closestPoint(p), path[1].closestPoint(p),
                     path[2].closestPoint(p)},
                    [&p](const Vector2& first, const Vector2& second) {
                      return (first - p).magnitudeSquared() <
                             (second - p).magnitudeSquared();
                    });
  }

  [[nodiscard]] std::string toStr() const {
    std::stringstream out;
    out << path[0].toStr() << ", " << path[1].toStr() << ", "
        << path[2].toStr();
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

  /**
   *
   * @param start
   * @param goal
   * @param radius
   * @tparam right If true, then the RSR path will be returned, otherwise the
   * LSL path will be returned.
   * @return
   */
  template <bool right>
  static DubinsPath getCSCOuterTangent(const PosVelState<T>& start,
                                       const PosVelState<T>& goal,
                                       const T& radius) {
    const Vector2 p1 = getCenter<right>(start, radius);
    const Vector2 p2 = getCenter<right>(goal, radius);
    const Vector2 v1 = p2 - p1;

    const T d = v1.magnitude();
    const Vector2 normal =
        (radius / d) * (getRot90Matrix<!right>() *
                        v1);  // extra brackets to reduce multiplications
    const Vector2 p1_tangent = p1 + normal;
    const Vector2 p2_tangent = p2 + normal;

    auto [start_angle, delta_angle] =
        getArcAngle<right>(start.pos - p1, normal);
    const DubinsCurve<T> c1{p1, radius, start_angle, delta_angle};
    const DubinsCurve<T> c2{p1_tangent, p2_tangent};
    std::tie(start_angle, delta_angle) =
        getArcAngle<right>(normal, goal.pos - p2);
    const DubinsCurve<T> c3{p2, radius, start_angle, delta_angle};
    return DubinsPath{{c1, c2, c3}};
  }

  /**
   *
   * @param start
   * @param goal
   * @param radius
   * @tparam right If true, then the RSL path will be returned, otherwise the
   * LSR path will be returned.
   * @return
   */
  template <bool right>
  static std::pair<bool, DubinsPath> getCSCInnerTangent(
      const PosVelState<T>& start, const PosVelState<T>& goal,
      const T& radius) {
    const Vector2 p1 = getCenter<right>(start, radius);
    const Vector2 p2 = getCenter<!right>(goal, radius);
    const Vector2 v1 = p2 - p1;

    const T d = v1.magnitude();
    if (d < 2 * radius) {
      // not possible to create RSL trajectory since the circles intersect,
      // return invalid DubinsPath
      return {false, DubinsPath{}};
    }
    const T cos = 2 * radius / d;
    const T sin = std::sqrt(1 - cos * cos);
    Matrix<T, 2, 2> rot{cos, -sin, sin, cos};
    if constexpr (!right)
      rot.transposeInPlace();  // transposing rotation matrix is the same as
                               // taking the inverse
    const Vector2 normal =
        (radius / d) * (rot * v1);  // extra brackets to reduce multiplications
    const Vector2 p1_tangent = p1 + normal;
    const Vector2 p2_tangent = p2 - normal;

    auto [start_angle, delta_angle] =
        getArcAngle<right>(start.pos - p1, normal);
    const DubinsCurve<T> c1{p1, radius, start_angle, delta_angle};
    const DubinsCurve<T> c2{p1_tangent, p2_tangent};
    std::tie(start_angle, delta_angle) =
        getArcAngle<!right>(-normal, goal.pos - p2);
    const DubinsCurve<T> c3{p2, radius, start_angle, delta_angle};
    return {true, DubinsPath{{c1, c2, c3}}};
  }

  /**
   *
   * @param start
   * @param goal
   * @param radius
   * @tparam right If true, then the RLR path will be returned, otherwise the
   * LRL path will be returned.
   * @return
   */
  template <bool right>
  static std::pair<bool, DubinsPath> getCCC(const PosVelState<T>& start,
                                            const PosVelState<T>& goal,
                                            const T& radius) {
    const Vector2 p1 = getCenter<right>(start, radius);
    const Vector2 p2 = getCenter<right>(goal, radius);
    const Vector2 v1 = p2 - p1;

    const T d = v1.magnitude();
    if (d >= 4 * radius) {
      // not possible to create RLR trajectory, return invalid DubinsPath
      // technically, if d >= 4 * radius then a RLR trajectory is possible,
      // however it will always be suboptimal
      return {false, DubinsPath{}};
    }
    const T h = std::sqrt(4 * radius * radius - d * d / 4);
    const Vector2 normal = (h / d) * (getRot90Matrix<!right>() * v1);
    const Vector2 p3 = p1 + v1 / 2 + normal;

    const Vector2 p_first_stop = (p1 + p3) / 2;
    const Vector2 p_second_stop = (p2 + p3) / 2;

    auto [start_angle, delta_angle] =
        getArcAngle<right>(start.pos - p1, p_first_stop - p1);
    const DubinsCurve<T> c1{p1, radius, start_angle, delta_angle};
    std::tie(start_angle, delta_angle) =
        getArcAngle<!right>(p_first_stop - p3, p_second_stop - p3);
    const DubinsCurve<T> c2{p3, radius, start_angle, delta_angle};
    std::tie(start_angle, delta_angle) =
        getArcAngle<right>(p_second_stop - p2, goal.pos - p2);
    const DubinsCurve<T> c3{p2, radius, start_angle, delta_angle};
    return {true, DubinsPath{{c1, c2, c3}}};
  }

  template <bool right>
  static Vector2 getCenter(const PosVelState<T>& state, const T& radius) {
    Vector2 normal = getRot90Matrix<right>() * state.vel;
    normal *= radius / normal.magnitude();
    return state.pos + normal;
  }

  /**
   * The start angle will be in the range [-pi, pi]
   * delta_angle will be in the range [-2 * pi, 2 * pi], where positive
   * indicates counterclockwise (turning left) The returned total angle
   * traversed will be positive if it is a left turn, and negative if it is a
   * right turn.
   * @return A std::pair consisting of the starting angle and the total angle
   * traversed
   */
  template <bool right>
  static std::pair<T, T> getArcAngle(const Vector2& start, const Vector2& end) {
    const T start_angle = std::atan2(start[1], start[0]);
    const T delta_angle = std::atan2(end[1], end[0]) - start_angle;
    if constexpr (right) {
      return {start_angle,
              delta_angle < 0 ? delta_angle : delta_angle - 2 * M_PI};
    } else {
      return {start_angle,
              delta_angle > 0 ? delta_angle : delta_angle + 2 * M_PI};
    }
  }

  template <bool right>
  static constexpr Matrix<T, 2, 2> getRot90Matrix() {
    if constexpr (right)
      return bmb_math::ROT_90_CW<T>;
    else
      return bmb_math::ROT_90_CCW<T>;
  }
};
