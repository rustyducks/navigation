#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <deque>
#include <vector>

#include "Navigation/Geometry/Point.h"

namespace rd {

struct PointOrientedSpeed {
  PointOriented point;
  double speed;
};

class Trajectory {
 public:
  Trajectory();
  Trajectory(const std::vector<PointOriented> &points);
  static Trajectory lissajouTrajectory(const PointOriented &robotPose, double tStep);
  /**
   * @brief Finds the closest point on the trajectory to the point given. The returned point is the linear interpolation of factor tOut
   * between the points at closestPrevIndex and closestPrevIndex + 1. The search stops if a point on the trajectory has a null speed.
   *
   * @param point The point to search for the closest point on the trajectory
   * @param tOut The interpolation factor
   * @param closestPrevIndex The index of the previous point of the trajectory
   * @return Point The closest point on the trajectory.
   * (Point = traj.at(closestPrevIndex) + tOut * (traj.at(closestPrevIndex + 1) - traj.at(closestPrevIndex)))
   */
  Point pointWithSpeedClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const;
  /**
   * @brief Finds the closest point on the trajectory to the given point. The search stops when the distance to the point increases (In case of trajectory crossings).
   *
   * @param point The point to search for the closest point on the trajectory
   * @param tOut The interpolation factor
   * @param closestPrevIndex The index of the previous point of the trajectory
   * @return Point The closest point on the trajectory
   * (Point = traj.at(closestPrevIndex) + tOut * (traj.at(closestPrevIndex + 1) - traj.at(closestPrevIndex)))
   */
  Point nextPointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const;

  /**
   * @brief Finds the point along the trajectory at 'distance' from 'pointStart'. First finds the closest point on the trajectory,
   * then browse the trajectory until distance or a point with null speed is reached.
   *
   * @param distance the distance along the trajectory
   * @param pointStart the starting point (will be projected on the trajectory)
   * @return Point the point on the trajectory at a distance from the projection of pointStart on the trajectory
   */
  Point pointAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex);

  void pop() { pointspeeds_.pop_front(); };
  const PointOrientedSpeed &at(size_t i) const;
  size_t size() const { return pointspeeds_.size(); };

 protected:
  std::deque<PointOrientedSpeed> pointspeeds_;
};
}  // namespace rd

#endif /* TRAJECTORY_H */
