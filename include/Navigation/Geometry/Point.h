#ifndef POINT_H
#define POINT_H

#include <eigen3/Eigen/Geometry>
#include <iostream>

namespace rd {

class Angle {
  friend class Point;

 public:
  Angle();
  Angle(double theta);
  Angle &operator-=(const Angle &rhs);
  Angle operator-(const Angle &rhs) const;
  Angle &operator-();
  Angle operator-() const;
  Angle &operator+=(const Angle &rhs);
  Angle operator+(const Angle &rhs) const;
  Angle &operator*=(const double s);
  Angle operator*(const double s) const;
  inline bool operator<(double angle) const { return *this < Angle(angle); };
  inline bool operator>(double angle) const { return *this > Angle(angle); };
  inline bool operator==(double angle) const { return *this == Angle(angle); };
  inline bool operator<(const Angle &rhs) const { return r_.smallestAngle() < rhs.r_.smallestAngle(); };
  inline bool operator>(const Angle &rhs) const { return r_.smallestAngle() > rhs.r_.smallestAngle(); };
  inline bool operator==(const Angle &rhs) const { return r_.smallestAngle() == rhs.r_.smallestAngle(); };

  double value() const { return r_.smallestAngle(); };
  inline double cos() const { return std::cos(r_.smallestAngle()); };
  inline double sin() const { return std::sin(r_.smallestAngle()); };

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
  virtual Point &operator+=(const Point &rhs);
  virtual Point operator+(const Point &rhs) const;

  inline friend std::ostream &operator<<(std::ostream &os, const Point &pt) {
    os << pt.p_.x() << ";" << pt.p_.y();
    return os;
  };

protected:
  Eigen::Vector2d p_;
};

class PointOriented : public Point {
public:
  PointOriented(double x, double y, double theta);
  const Angle &theta() const { return a_; };

  inline friend std::ostream &operator<<(std::ostream &os, const PointOriented &pt) {
    os << pt.p_.x() << ";" << pt.p_.y() << ";" << pt.a_;
    return os;
  };
  virtual PointOriented &operator+=(const PointOriented &rhs);
  virtual PointOriented operator+(const PointOriented &rhs) const;

private:
  Angle a_;
};

} // namespace rd

#endif /* POINT_H */
