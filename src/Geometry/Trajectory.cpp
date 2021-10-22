#include "Navigation/Geometry/Trajectory.h"

#include <limits>

#include "Navigation/Communication/Ivy.h"

namespace rd {
Trajectory::Trajectory() : pointspeeds_({}) {}

Trajectory::Trajectory(const std::vector<PointOriented> &points) {
  for (PointOriented p : points) {
    pointspeeds_.push_back({p, 0.0});
  }
  computeSpeeds();
}

Trajectory Trajectory::lissajouTrajectory(const PointOriented &robotPose, const double tStep) {
  double theta = 0.;
  std::vector<PointOriented> pts;
  while (theta < 2 * M_PI) {
    pts.emplace_back(robotPose.x() + 500 * std::sin(theta), robotPose.y() + 500 * std::sin(2 * theta), 0.0);
    theta += tStep;
  }
  return Trajectory(pts);
}

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

Point Trajectory::pointAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) {
  double t;
  size_t previousIndex;
  // Ivy::getInstance().sendPoint(4, pointStart);
  Point proj = pointWithSpeedClosestTo(pointStart, t, previousIndex);
  // Ivy::getInstance().sendPoint(6, proj);

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
  // Ivy::getInstance().sendPoint(5, goal);
  previousClosestIndex = browsingTraj;
  return goal;
}

double Trajectory::mengerCurvature(const size_t i) const {
  // bad idea... Should use the angle instead: the curvature depends on the length between considered points. Does not work for polyline
  if (pointspeeds_.size() < 2) {
    return 0.0;
  }
  if (i == 0 || i >= pointspeeds_.size() - 1) {
    return 0.0;
  }
  const PointOriented &x = pointspeeds_.at(i - 1).point;
  const PointOriented &y = pointspeeds_.at(i).point;
  const PointOriented &z = pointspeeds_.at(i + 1).point;
  if (x == y || y == z || x == z) {
    return 0.0;
  }
  const Angle xyzAngle = (x - y).angleBetweenVectors(z - y);
  return 2. * xyzAngle.sin() / (x - z).norm();
}

void Trajectory::computeSpeeds() {
  if (pointspeeds_.size() < 2) {
    return;
  }
  pointspeeds_.front().speed = 0.;
  pointspeeds_.back().speed = 0.;
  for (size_t i = 1; i < pointspeeds_.size() - 1; i++) {
    const PointOriented &x = pointspeeds_.at(i - 1).point;
    const PointOriented &y = pointspeeds_.at(i).point;
    const PointOriented &z = pointspeeds_.at(i + 1).point;
    const Angle &angle = (y - x).angleBetweenVectors(z - y);
    const double a = 100. / (M_PI / 30 - M_PI / 8);  // a = max_speed / (slowing_starting_angle - stop_angle)
    const double b = -a * M_PI / 8;                  // b = a * stop_angle
    pointspeeds_.at(i).speed = std::min(100., std::max(0., a * angle.value() + b));
  }

  for (int i = pointspeeds_.size() - 2; i >= 0; i--) {
    // Browse the trajectory in reverse to find if the speed at a point dictates the speed at a previous one
    const PointOrientedSpeed &next = pointspeeds_.at(i + 1);
    const PointOrientedSpeed &current = pointspeeds_.at(i);
    const double dist = current.point.distanceTo(next.point);
    const double maxSpeedAtMaxDecel = std::sqrt(next.speed * next.speed + 2. * 10. * dist);  // sqrt(v0**2 + 2 * maxAcc * distToTravel)
    pointspeeds_.at(i).speed = std::min(pointspeeds_.at(i).speed, maxSpeedAtMaxDecel);
    // Speed is min between max speed due to traj angle and max possible speed to be at the next point at the right speed
  }
}

}  // namespace rd