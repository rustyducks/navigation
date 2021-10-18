#include "Navigation/LinearControl.h"

namespace rd {
LinearControl::LinearControl() {}

Speed LinearControl::computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) {
  Point rp = targetPoint_.point - robotPose;
  Point robotHeading(robotPose.theta().cos(), robotPose.theta().sin());
  double robotLSpeed = robotSpeed.linearSpeed();
  double rpDist = rp.norm();
  double rpDotHeading = rp.dot(robotHeading);
  double acceleration = 0.;
  if (rpDotHeading < 0) {
    acceleration = -param<double>(MAX_LINEAR_ACCELERATION);
  } else {
    acceleration = param<double>(MAX_LINEAR_ACCELERATION);
  }
  double mul = param<bool>(IS_HOLONOMIC) ? 1.0 : std::abs(rpDotHeading / rpDist);
  if (rpDist < 3 * param<double>(ADMITTED_LINEAR_POSITION_ERROR)) {
    mul *= 0.2;
  }
  double commandSpeed = robotLSpeed + acceleration * dt * mul;
  if (std::abs(commandSpeed) > targetPoint_.speed) {
    double timeToSpeed = std::abs((robotLSpeed - targetPoint_.speed) / param<double>(MAX_LINEAR_ACCELERATION));
    // Factor is needed, else the length is largely underestimated. Probably because of control rate?
    double lengthToSpeed = param<double>(LINEAR_CONTROL_STOP_DISTANCE_FACTOR) * (robotLSpeed * timeToSpeed - 0.5 * acceleration * timeToSpeed * timeToSpeed);
    Point robot2SpeedPoint(lengthToSpeed * robotPose.theta().cos(),
                           lengthToSpeed * robotPose.theta().sin());  // TODO(guilhembn): This may not translate for holonomic robot
    Point speedPoint = robot2SpeedPoint + robotPose;
    Point pSpeed = speedPoint - targetPoint_.point;
    double plannedSpeedError = pSpeed.norm();
    if (plannedSpeedError <= param<double>(ADMITTED_LINEAR_POSITION_ERROR) || pSpeed.dot(-rp) < 0) {
      // We will get at the target point with the right speed, or overshoot it. We need to decelerate
      double speedDiff = acceleration * dt;
      if (std::abs(speedDiff) >= std::abs(robotLSpeed)) {
        commandSpeed = 0.;
      } else {
        commandSpeed = robotLSpeed - speedDiff;
      }
    } else if (plannedSpeedError <= 3 * param<double>(ADMITTED_LINEAR_POSITION_ERROR) && std::abs(robotLSpeed) > 10.) {
      commandSpeed = robotLSpeed;
    }
  }
  commandSpeed = std::min(param<double>(MAX_LINEAR_SPEED), std::max(-param<double>(MAX_LINEAR_SPEED), commandSpeed));
  if (param<bool>(IS_HOLONOMIC)) {
    Point robot2target = targetPoint_.point.transformIn(robotPose);
    Angle robot2targetAngle = robot2target.polarAngle();
    return Speed(commandSpeed * robot2targetAngle.cos(), commandSpeed * robot2targetAngle.sin(), 0.0);
  } else {
    return Speed(commandSpeed, 0.0, 0.0);  // We assume the direction will be handled via steering
  }
}
}  // namespace rd