#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <deque>
#include <vector>

#include "Navigation/Geometry/Point.h"

namespace rd {

class Trajectory;

class PointOrientedSpeed : public PointOriented {
 public:
  PointOrientedSpeed() : PointOriented(), speed_(0.) {}
  PointOrientedSpeed(const PointOriented &point, const double speed) : PointOriented(point), speed_(speed) {}
  double speed() const { return speed_; }

 protected:
  double speed_;
};

class Path {
 public:
  Path();
  Path(const std::vector<PointOriented> &points);
  static Path lissajouPath(const PointOriented &robotPose, const size_t steps);

  /**
   * @brief Finds the closest point on the trajectory to the point given. The returned point is the linear interpolation of factor tOut
   * between the points at closestPrevIndex and closestPrevIndex + 1.
   *
   * @param point The point to search for the closest point on the trajectory
   * @param tOut The interpolation factor
   * @param closestPrevIndex The index of the previous point of the trajectory
   * @return Point The closest point on the trajectory.
   * (Point = traj.at(closestPrevIndex) + tOut * (traj.at(closestPrevIndex + 1) - traj.at(closestPrevIndex)))
   */
  Point pointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const;
  /**
   * @brief Finds the closest point on the trajectory to the given point. The search stops when the distance to the point increases (In case of trajectory
   * crossings).
   *
   * @param point The point to search for the closest point on the trajectory
   * @param tOut The interpolation factor
   * @param closestPrevIndex The index of the previous point of the trajectory
   * @return Point The closest point on the trajectory
   * (Point = traj.at(closestPrevIndex) + tOut * (traj.at(closestPrevIndex + 1) - traj.at(closestPrevIndex)))
   */
  Point nextPointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const;
  Point pointAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const;
  Point pointAtBackwardDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const;

  /* Returns the trajectory length between the projection of a and the projection of b along the trajectory.
   */
  double distanceBetween(const Point &a, const Point &b) const;

  double mengerCurvature(const size_t i, const double distance) const;

  virtual void pop() { points_.pop_front(); }
  const PointOriented at(size_t i) const;
  size_t size() const { return points_.size(); }

  Trajectory computeSpeeds() const;

 protected:
  std::deque<PointOriented> points_;
};

class Trajectory : public Path {
 public:
  Trajectory();
  Trajectory(const Path &path, const std::vector<double> &speeds);

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
   * @brief Finds the point along the trajectory at 'distance' from 'pointStart'. First finds the closest point on the trajectory,
   * then browse the trajectory until distance or a point with null speed is reached.
   *
   * @param distance the distance along the trajectory
   * @param pointStart the starting point (will be projected on the trajectory)
   * @return Point the point on the trajectory at a distance from the projection of pointStart on the trajectory, along with an interpolated speed
   */
  Point pointWithSpeedAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const;
  Point pointWithSpeedAtBackwardDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const;

  virtual void pop() override {
    points_.pop_front();
    speeds_.pop_front();
  }
  const PointOrientedSpeed at(size_t i) const;

 protected:
  std::deque<double> speeds_;
};
}  // namespace rd

#endif /* TRAJECTORY_H */
