#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "Navigation/Geometry/Point.h"
#include <deque>
#include <vector>

namespace rd {

struct PointOrientedSpeed {
  PointOriented point;
  double speed;
};

class Trajectory {
public:
  Trajectory();
  Trajectory(const std::vector<PointOriented> &points);

  void pop() { pointspeeds_.pop_front(); };
  const PointOrientedSpeed &at(size_t i) const;
  size_t size() const { return pointspeeds_.size(); };

protected:
  std::deque<PointOrientedSpeed> pointspeeds_;
};
} // namespace rd

#endif /* TRAJECTORY_H */
