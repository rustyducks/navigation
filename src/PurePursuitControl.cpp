#include "Navigation/PurePursuitControl.h"

#include "Navigation/Communication/Ivy.h"

namespace rd {
PurePursuitControl::PurePursuitControl() : trajectory_() {
  linearControl_.setParam(IS_HOLONOMIC, false);
  rotationControl_.setParam(MAX_ROTATIONAL_SPEED, 1.0);  // Todo (guilhembn): Do not pass parameters like so...
  rotationControl_.setParam(MAX_ROTATIONAL_ACCELERATION, 0.1);
  rotationControl_.setParam(ADMITTED_ANGLE_POSITION_ERROR, 0.05);
  linearControl_.setParam(ParamName::MAX_LINEAR_ACCELERATION, 10.);
  linearControl_.setParam(ParamName::MAX_LINEAR_SPEED, 100.);
  linearControl_.setParam(ADMITTED_LINEAR_POSITION_ERROR, 5.);
  linearControl_.setParam(LINEAR_CONTROL_STOP_DISTANCE_FACTOR, 2.);
}

Speed PurePursuitControl::computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) {
  Angle targetAngle;
  switch (state_) {
    case PurePursuitState::IDLE:
      return Speed(0., 0., 0.);
      break;
    case PurePursuitState::FIRST_ROTATION:

      targetAngle = robotPose.angleTo(trajectory_.at(1).point);  // trajectory.at(0) is past point, in order for the interpolation to work
      if (std::abs(robotSpeed.vtheta()) <= 0.01 &&
          std::abs((targetAngle - robotPose.theta()).value()) < 0.05) {  // Todo (guilhembn): Find a way to pass parameters
        state_ = PurePursuitState::CRUISING;
        return cruising(robotPose, robotSpeed, dt);
      } else {
        rotationControl_.setTargetAngle(targetAngle);
        return rotationControl_.computeSpeed(robotPose, robotSpeed, dt);
      }
      break;

    case PurePursuitState::CRUISING:

      return cruising(robotPose, robotSpeed, dt);
      break;
    case PurePursuitState::LAST_ROTATION:
      if (std::abs(robotSpeed.vtheta()) <= 0.01 && std::abs((trajectory_.at(0).point.theta() - robotPose.theta()).value()) < 0.05) {
        trajectory_.pop();
        assert(trajectory_.size() == 0);
        state_ = PurePursuitState::IDLE;
        return Speed(0.0, 0.0, 0.0);
      } else {
        return rotationControl_.computeSpeed(robotPose, robotSpeed, dt);
      }
      break;
    default:
      throw std::runtime_error("Invalid state in PurePursuitController.");
  }
  return Speed(0.0, 0.0, 0.0);
}

Speed PurePursuitControl::cruising(const PointOriented& robotPose, const Speed& robotSpeed, double dt) {
  PointOriented nextTrajPoint = trajectory_.at(trajectoryCurrentIndex_).point;  // trajectory.at(0) is past point, in order for the interpolation to work
  double nextTrajSpeed = trajectory_.at(trajectoryCurrentIndex_).speed;
  double positionError = robotPose.distanceTo(nextTrajPoint);
  Ivy::getInstance().sendPoint(5, nextTrajPoint);
  if (positionError < param<double>(ADMITTED_LINEAR_POSITION_ERROR)) {
    // If we are not too far
    if (trajectoryCurrentIndex_ == trajectory_.size() - 1) {
      // If it is the last point of the trajectory, start final rotation
      for (size_t i = 0; i < trajectoryCurrentIndex_; i++) {
        trajectory_.pop();
      }
      trajectoryCurrentIndex_ = 1;
      state_ = PurePursuitState::LAST_ROTATION;
      rotationControl_.setTargetAngle(nextTrajPoint.theta());
      return rotationControl_.computeSpeed(robotPose, robotSpeed, dt);
    } else if (nextTrajSpeed < 0.1) {
      // If it is not the last point, but we should stop, start a new initial rotation
      for (size_t i = 0; i < trajectoryCurrentIndex_; i++) {
        trajectory_.pop();
      }
      trajectoryCurrentIndex_ = 1;
      state_ = PurePursuitState::FIRST_ROTATION;
      Angle targetAngle = robotPose.angleTo(trajectory_.at(1).point);
      rotationControl_.setTargetAngle(targetAngle);
      return rotationControl_.computeSpeed(robotPose, robotSpeed, dt);
    } else {
      // Go to the next point in cruising
      for (size_t i = 0; i < trajectoryCurrentIndex_; i++) {
        trajectory_.pop();
      }
      trajectoryCurrentIndex_ = 1;
      return purePursuit(robotPose, robotSpeed, dt);
    }
  } else {
    return purePursuit(robotPose, robotSpeed, dt);
  }
}

Speed PurePursuitControl::purePursuit(const PointOriented& robotPose, const Speed& robotSpeed, double dt) {
  size_t previousClosestIndex;
  Point goal = trajectory_.pointAtDistanceFrom(param<double>(PURE_PURSUIT_LOOKAHEAD_DISTANCE), robotPose, previousClosestIndex);
  trajectoryCurrentIndex_ = previousClosestIndex + 1;
  linearControl_.setTargetPoint(trajectory_.at(trajectoryCurrentIndex_));
  Speed linear = linearControl_.computeSpeed(robotPose, robotSpeed, dt);
  double vx = linear.linearSpeed();  // Only the magnitude of the speed interest us, the rest is handled via steering
  Point robot2Goal = goal.transformIn(robotPose);
  double curvature = 2 * robot2Goal.y() / robot2Goal.squaredNorm();
  double vtheta = vx * curvature;
  if (std::abs(vtheta) > param<double>(MAX_ROTATIONAL_SPEED)) {
    vtheta = std::min(param<double>(MAX_ROTATIONAL_SPEED), std::max(-param<double>(MAX_ROTATIONAL_SPEED), vtheta));
    // vtheta = std::min(robotSpeed.vtheta() + param<double>(MAX_ROTATIONAL_ACCELERATION) * dt,
    //                  std::max(robotSpeed.vtheta() - param<double>(MAX_ROTATIONAL_ACCELERATION) * dt, vtheta));
    assert(curvature != 0.0);  // curvature can't be 0, else, vtheta would have been 0, and not exceeding max speed.
    vx = vtheta / curvature;
  }
  return Speed(vx, 0.0, vtheta);
}
}  // namespace rd
