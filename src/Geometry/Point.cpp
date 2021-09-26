#include "Navigation/Geometry/Point.h"

namespace rd {

Angle::Angle() : r_(0.0) {}

Angle::Angle(double theta) : r_(theta) { center(); }

void Angle::center() { r_.angle() = r_.smallestAngle(); }

Angle &Angle::operator-=(const Angle &rhs) {
  r_ *= rhs.r_.inverse();
  center();
  return *this;
}

Angle Angle::operator-(const Angle &rhs) const {
  Angle a(*this);
  a -= rhs;
  return a;
}

Angle &Angle::operator-() {
  r_ = r_.inverse();
  return *this;
}

Angle Angle::operator-() const {
  Angle a(*this);
  return -a;
}

Angle &Angle::operator+=(const Angle &rhs) {
  r_ *= rhs.r_;
  center();
  return *this;
}

Angle Angle::operator+(const Angle &rhs) const {
  Angle a(*this);
  a += rhs;
  return a;
}

Angle Point::angleTo(const Point &pt) const {
  const Eigen::Vector2d tmp = pt.p_ - p_;
  return Angle(std::atan2(tmp.y(), tmp.x()));
}

Point::Point(double x, double y) : p_(x, y) {}

Point &Point::operator+=(const Point &rhs) {
  p_ += rhs.p_;
  return *this;
}

Point Point::operator+(const Point &rhs) const {
  Point tmp(*this);
  tmp += rhs;
  return tmp;
}

PointOriented::PointOriented(double x, double y, double theta) : Point(x, y), a_(theta) {}

PointOriented &PointOriented::operator+=(const PointOriented &rhs) {
  p_ += rhs.p_;
  a_ += rhs.a_;
  return *this;
}

PointOriented PointOriented::operator+(const PointOriented &rhs) const {
  PointOriented tmp(*this);
  tmp += rhs;
  return tmp;
}

}  // namespace rd