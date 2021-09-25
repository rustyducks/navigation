#ifndef SPEED_H
#define SPEED_H

#include <eigen3/Eigen/Geometry>

namespace rd {
class Speed {
public:
  Speed(double vx, double vy, double vtheta);

  double vx() const { return linearSpeed_.x(); };
  double vy() const { return linearSpeed_.y(); };
  double vtheta() const { return rotationalSpeed_; };

protected:
  Eigen::Vector2d linearSpeed_;
  double rotationalSpeed_;
};
} // namespace rd

#endif /* SPEED_H */
