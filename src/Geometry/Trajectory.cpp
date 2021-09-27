#include "Navigation/Geometry/Trajectory.h"

#include <limits>

namespace rd {
Trajectory::Trajectory() : pointspeeds_({}) {}

Trajectory::Trajectory(const std::vector<PointOriented> &) {}

const PointOrientedSpeed &Trajectory::at(size_t i) const { return pointspeeds_.at(i); }

Point Trajectory::pointWithSpeedClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const {
  double distMin = std::numeric_limits<double>::max();
  double tMin = 0.;
  size_t iMin = -1;
  Point pointMin;
  for (size_t i = 0; i < size() - 1; i++) {
    double t;
    Point pt = point.closestPointBetween(pointspeeds_.at(i).point, pointspeeds_.at(i + 1).point, t);
    double dist = pt.squaredDistanceTo(point);
    if (dist < distMin) {
      distMin = dist;
      tMin = t;
      iMin = i;
      pointMin = pt;
    }
    if (i != 0 && pointspeeds_.at(i).speed == 0.0) {
      break;
    }
  }
  tOut = tMin;
  closestPrevIndex = iMin;
  return pointMin;
}

Point Trajectory::nextPointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const {
  double distMin = std::numeric_limits<double>::max();
  double tMin = 0.;
  size_t iMin = -1;
  Point pointMin;
  for (size_t i = 0; i < size() - 1; i++) {
    double t;
    Point pt = point.closestPointBetween(pointspeeds_.at(i).point, pointspeeds_.at(i + 1).point, t);
    double dist = pt.squaredDistanceTo(point);
    if (dist < distMin) {
      distMin = dist;
      tMin = t;
      iMin = i;
      pointMin = pt;
    }
    if (dist > distMin) {
      break;
    }
  }
  tOut = tMin;
  closestPrevIndex = iMin;
  return pointMin;
}

Point Trajectory::pointAtDistanceFrom(const double distance, const Point &pointStart) {
  double t;
  size_t previousIndex;
  Point proj = nextPointClosestTo(pointStart, t, previousIndex);

  double distanceLeft = distance;
  double pathLen = pointspeeds_.at(previousIndex + 1).point.distanceTo(proj);
  size_t browsingTraj = previousIndex;
  while (distanceLeft > pathLen) {  // while the distance left is greater than the length of a full segment
    if (pointspeeds_.at(browsingTraj + 1).speed == 0.0) {
      // If the robot must stop on this point, do not explore the rest, manage to get it there
      t = 0.;
      distanceLeft = (pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point).norm();
      break;
    }
    t = 0.;
    browsingTraj++;
    distanceLeft -= pathLen;
    pathLen = (pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point).norm();
  }
  Point ab = pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point;
  double tGoal = t + distanceLeft / ab.norm();
  Point goal = (Point)pointspeeds_.at(browsingTraj).point + ab * tGoal;
  return goal;
}

}  // namespace rd