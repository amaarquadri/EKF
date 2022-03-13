#pragma once

#include <bmb_math/Utility.h>
#include <bmb_math/Vector.h>
#include <cmath>
#include <sstream>
#include <string>
#include <variant>

template <typename T>
struct DubinsCurve {
  struct Line {
    Vector<T, 2> start_pos;
    Vector<T, 2> end_pos;
  };

  struct Circle {
    Vector<T, 2> center;
    T radius;
    T start_angle;
    T delta_angle;
  };

  std::variant<Circle, Line> data;

  bool isCircle() const { return std::holds_alternative<Circle>(data); }

  const Circle& getCircle() const { return std::get<Circle>(data); }

  Circle& getCircle() { return std::get<Circle>(data); }

  bool isLine() const { return std::holds_alternative<Line>(data); }

  const Line& getLine() const { return std::get<Line>(data); }

  Line& getLine() { return std::get<Line>(data); }

  DubinsCurve(const Vector<T, 2>& center, const T& radius, const T& start_angle,
              const T& delta_angle)
      : data(Circle{center, radius, start_angle, delta_angle}) {}

  DubinsCurve(const Vector<T, 2>& start_pos, const Vector<T, 2>& end_pos)
      : data(Line{start_pos, end_pos}) {}

  DubinsCurve(const DubinsCurve<T>& other) { (*this) = other; }

  /**
   * This constructor will create this DubinsCurve with unspecified data inside
   * of it. It is the caller's responsibility to handle this.
   */
  DubinsCurve(){
      // cannot use = default to define this constructor because then it will
      // be implicitly deleted
  };

  DubinsCurve& operator=(const DubinsCurve<T>& other) {
    data = other.data;
    return *this;
  }

  [[nodiscard]] T getLength() const {
    if (isCircle()) {
      const Circle& circle = getCircle();
      return circle.radius * std::fabs(circle.delta_angle);
    } else {
      const Line& line = getLine();
      return (line.start_pos - line.end_pos).magnitude();
    }
  }

  /**
   * Return value is unspecified if !is_turning
   */
  [[nodiscard]] bool isRightTurn() const { return getCircle().delta_angle < 0; }

  /**
   * @return The point on this DubinsCurve closest to the specified point
   */
  [[nodiscard]] Vector<T, 2> closestPoint(const Vector<T, 2>& p) const {
    using bmb_math::getRotationMatrix;

    if (isLine()) {
      const Line& line = getLine();
      const Vector2 d = line.end_pos - line.start_pos;
      const T projection_fraction = (p - line.start_pos).dot(d) / d.dot(d);
      if (projection_fraction < 0) return line.start_pos;
      if (projection_fraction > 1) return line.end_pos;
      return line.start_pos + projection_fraction * d;
    }

    // otherwise, we are turning
    const Circle& circle = getCircle();
    const Vector2 disp = p - circle.center;
    T relative_angle = std::atan2(disp[1], disp[0]) - circle.start_angle;
    if (isRightTurn()) {
      // remap angle to [-2 * PI, 0]
      if (relative_angle > 0)
        relative_angle -= 2 * M_PI;
      else if (relative_angle < -2 * M_PI)
        relative_angle += 2 * M_PI;

      if (relative_angle > circle.delta_angle) {
        // closest point on the circle to p is within the delta_angle range
        return circle.center + disp * (circle.radius / disp.magnitude());
      } else if (relative_angle > circle.delta_angle / 2 - M_PI) {
        // closest point is the end point
        return circle.center +
               getRotationMatrix(circle.start_angle + circle.delta_angle) *
                   Vector2{circle.radius, 0};
      } else {
        // closest point is the start point
        return circle.center + getRotationMatrix(circle.start_angle) *
                                   Vector2{circle.radius, 0};
      }
    } else {
      // remap angle to [0, 2 * PI]
      if (relative_angle > 2 * M_PI)
        relative_angle -= 2 * M_PI;
      else if (relative_angle < 0)
        relative_angle += 2 * M_PI;

      if (relative_angle < circle.delta_angle) {
        // closest point on the circle to p is within the delta_angle range
        return circle.center + disp * (circle.radius / disp.magnitude());
      } else if (relative_angle < circle.delta_angle / 2 + M_PI) {
        // closest point is the end point
        return circle.center +
               getRotationMatrix(circle.start_angle + circle.delta_angle) *
                   Vector2{circle.radius, 0};
      } else {
        // closest point is the start point
        return circle.center + getRotationMatrix(circle.start_angle) *
                                   Vector2{circle.radius, 0};
      }
    }
  }

  [[nodiscard]] std::string toStr() const {
    std::stringstream out;
    if (isLine())
      out << "Straight for: " << getLength() << "m";
    else {
      const Circle& circle = getCircle();
      out << "Turn " << ((circle.delta_angle < 0) ? "right" : "left")
          << " for: " << std::fabs(circle.delta_angle) << "deg";
    }
    return out.str();
  }

  template <typename OStream>
  void toCSV(OStream& out) const {
    if (isLine()) {
      for (size_t i = 0; i < NUM_SAMPLES; i++) {
        const Line& line = getLine();
        const Vector<T, 2> pos =
            (static_cast<T>(NUM_SAMPLES - i) * line.start_pos +
             static_cast<T>(i) * line.end_pos) /
            static_cast<T>(NUM_SAMPLES);
        out << pos[0] << "," << pos[1] << std::endl;
      }
    } else {
      for (size_t i = 0; i < NUM_SAMPLES; i++) {
        const Circle& circle = getCircle();
        const T theta = circle.start_angle +
                        circle.delta_angle * (i / static_cast<T>(NUM_SAMPLES));
        const Vector<T, 2> pos =
            circle.center +
            bmb_math::getRotationMatrix(theta) * Vector2{circle.radius, 0};
        out << pos[0] << "," << pos[1] << std::endl;
      }
    }
  }

 private:
  using Vector2 = Vector<T, 2>;
  static constexpr size_t NUM_SAMPLES = 100;
};