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

Point::Point(double x, double y) : p_(x, y) {}

std::ostream &operator<<(std::ostream &os, const Point &pt) {
  os << pt.p_.x() << ";" << pt.p_.y();
  return os;
}

Angle Point::angleTo(const Point &pt) const {
  const Eigen::Vector2d tmp = pt.p_ - p_;
  return Angle(std::atan2(tmp.y(), tmp.x()));
}

PointOriented::PointOriented(double x, double y, double theta) : Point(x, y), a_(theta) {}

std::ostream &operator<<(std::ostream &os, const PointOriented &pt) {
  os << pt.p_.x() << ";" << pt.p_.y() << ";" << pt.a_ << "rad";
  return os;
}

} // namespace rd