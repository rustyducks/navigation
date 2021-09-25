#ifndef POINT_H
#define POINT_H

#include <eigen3/Eigen/Geometry>
#include <iostream>

namespace rd {

class Angle {
public:
  Angle();
  Angle(double theta);
  Angle &operator-=(const Angle &rhs);
  Angle operator-(const Angle &rhs) const;
  Angle &operator-();
  Angle operator-() const;

  double value() { return r_.smallestAngle(); };

  inline friend std::ostream &operator<<(std::ostream &os, const Angle &pt) {
    os << pt.r_.smallestAngle() << "rad";
    return os;
  };

protected:
  void center();
  Eigen::Rotation2Dd r_;
};

class Point {
public:
  Point(double x, double y);
  virtual Angle angleTo(const Point &pt) const;
  double x() const { return p_.x(); };
  double y() const { return p_.y(); };

  friend std::ostream &operator<<(std::ostream &os, const Point &pt);

protected:
  Eigen::Vector2d p_;
};

class PointOriented : public Point {
public:
  PointOriented(double x, double y, double theta);
  const Angle &theta() const { return a_; };

  friend std::ostream &operator<<(std::ostream &os, const PointOriented &pt);

private:
  Angle a_;
};

} // namespace rd

#endif /* POINT_H */
