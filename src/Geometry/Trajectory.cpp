#include "Navigation/Geometry/Trajectory.h"

#include <limits>

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

Point Trajectory::pointAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex, bool withSpeed) const {
  double t;
  size_t previousIndex;
  Point proj = pointWithSpeedClosestTo(pointStart, t, previousIndex);

  double distanceLeft = distance;
  double pathLen = pointspeeds_.at(previousIndex + 1).point.distanceTo(proj);
  size_t browsingTraj = previousIndex;
  while (distanceLeft > pathLen) {  // while the distance left is greater than the length of a full segment
    if (withSpeed && pointspeeds_.at(browsingTraj + 1).speed == 0.0) {
      // If the robot must stop on this point, do not explore the rest, manage to get it there
      t = 0.;
      distanceLeft = (pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point).norm();
      break;
    }
    t = 0.;
    browsingTraj += 1;
    distanceLeft -= pathLen;
    pathLen = (pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point).norm();
  }
  Point ab = pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point;
  double tGoal = t + distanceLeft / ab.norm();
  Point goal = (Point)pointspeeds_.at(browsingTraj).point + ab * tGoal;
  previousClosestIndex = browsingTraj;
  return goal;
}

Point Trajectory::pointAtBackwardDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex, bool withSpeed) const {
  double t;
  size_t previousIndex;
  Point proj = pointWithSpeedClosestTo(pointStart, t, previousIndex);
  double distanceLeft = std::abs(distance);
  double pathLen = pointspeeds_.at(previousIndex).point.distanceTo(proj);
  size_t browsingTraj = previousIndex;
  while (distanceLeft > pathLen) {  // while the distance left is greater than the length of a full segment
    if (withSpeed && pointspeeds_.at(browsingTraj).speed == 0.0) {
      // If the robot must stop on this point, do not explore the rest, manage to get it there
      t = 1.;
      distanceLeft = (pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point).norm();
      break;
    }
    t = 1.;
    browsingTraj -= 1;
    distanceLeft -= pathLen;
    pathLen = (pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point).norm();
  }
  Point ab = pointspeeds_.at(browsingTraj + 1).point - pointspeeds_.at(browsingTraj).point;
  double tGoal = t - distanceLeft / ab.norm();
  Point goal = (Point)pointspeeds_.at(browsingTraj).point + ab * tGoal;
  previousClosestIndex = browsingTraj;
  return goal;
}

double Trajectory::distanceBetween(const Point &a, const Point &b) const {
  double t1, t2;
  size_t i1, i2;
  Point proj1 = pointWithSpeedClosestTo(a, t1, i1);
  Point proj2 = pointWithSpeedClosestTo(b, t2, i2);
  if (i1 == i2) {
    // Both projections are on the same segment
    return proj1.distanceTo(proj2);
  }
  if (i1 < i2) {
    double dist = proj1.distanceTo(pointspeeds_.at(i1 + 1).point);
    for (size_t i = i1 + 1; i < i2; i++) {
      dist += pointspeeds_.at(i).point.distanceTo(pointspeeds_.at(i + 1).point);
    }
    dist += proj2.distanceTo(pointspeeds_.at(i2).point);
    return dist;
  } else {
    double dist = proj2.distanceTo(pointspeeds_.at(i2 + 1).point);
    for (size_t i = i2 + 1; i < i1; i++) {
      dist += pointspeeds_.at(i).point.distanceTo(pointspeeds_.at(i + 1).point);
    }
    dist += proj1.distanceTo(pointspeeds_.at(i1).point);
    return dist;
  }
}

double Trajectory::mengerCurvature(const size_t i, const double computeDistance) const {
  // bad idea... Should use the angle instead: the curvature depends on the length between considered points. Does not work for polyline
  // Update: Actually, if we keep a specified distance to the point (before and after), it would return a curvature not depending on the distance between
  // consecutive points
  if (pointspeeds_.size() < 2) {
    return 0.0;
  }
  if (i == 0 || i >= pointspeeds_.size() - 1) {
    return 0.0;
  }
  const PointOriented &y = pointspeeds_.at(i).point;
  size_t k;
  Point x = pointAtBackwardDistanceFrom(computeDistance, y, k, false);
  Point z = pointAtDistanceFrom(computeDistance, y, k, false);
  double xy = distanceBetween(x, y);
  double yz = distanceBetween(y, z);

  if (xy < computeDistance - 0.001 && xy <= yz) {
    z = pointAtDistanceFrom(xy, y, k, false);
    yz = distanceBetween(y, z);
  } else if (yz < computeDistance - 0.001 && yz <= xy) {
    x = pointAtBackwardDistanceFrom(yz, y, k, false);
    xy = distanceBetween(x, y);
  }

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
    const double curvature = mengerCurvature(i, 20.);
    if (curvature < 0.01) {
      pointspeeds_.at(i).speed = 150.;  // Trajectory is straight, full speed
    } else {
      double centripetalAccMaxSpeed = std::sqrt(50. / curvature);  // Centripetal acceleration = v**2 * curvature
      double maxRotSpeed = 1.8 / curvature;                        // maxRotSpeed / curvature = vx  (vtheta = vx * c)
      double minSpeed = std::min(maxRotSpeed, centripetalAccMaxSpeed);
      pointspeeds_.at(i).speed = std::min(150., std::max(0., minSpeed));
    }
  }

  for (int i = pointspeeds_.size() - 2; i >= 0; i--) {
    // Browse the trajectory in reverse to find if the speed at a point dictates the speed at a previous one
    const PointOrientedSpeed &next = pointspeeds_.at(i + 1);
    const PointOrientedSpeed &current = pointspeeds_.at(i);
    const double dist = current.point.distanceTo(next.point);
    const double maxSpeedAtMaxDecel = std::sqrt(next.speed * next.speed + 2. * 50. * dist);  // sqrt(v0**2 + 2 * maxAcc * distToTravel)
    pointspeeds_.at(i).speed = std::min(pointspeeds_.at(i).speed, maxSpeedAtMaxDecel);
    // Speed is min between max speed due to traj angle and max possible speed to be at the next point at the right speed
  }
}

}  // namespace rd