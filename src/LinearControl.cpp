#include "Navigation/LinearControl.h"

#include "Navigation/Communication/Ivy.h"
#include "Navigation/Parameters.h"

namespace rd {
LinearControl::LinearControl() {}

Speed LinearControl::computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) {
  Point rp = targetPoint_ - robotPose;
  Point robotHeading(robotPose.theta().cos(), robotPose.theta().sin());
  double robotLSpeed = robotSpeed.linearSpeed();
  double rpDist = rp.norm();
  double rpDotHeading = rp.dot(robotHeading);
  double acceleration = 0.;
  if (rpDotHeading < 0) {
    acceleration = -MAX_LINEAR_ACCELERATION;
  } else {
    acceleration = MAX_LINEAR_ACCELERATION;
  }
  double mul = IS_HOLONOMIC ? 1.0 : std::abs(rpDotHeading / rpDist);
  if (rpDist < 3 * ADMITTED_LINEAR_POSITION_ERROR) {
    mul *= 0.2;
  }
  double commandSpeed = robotLSpeed + acceleration * dt * mul;
  // std::cout << "command speed: " << commandSpeed << std::endl;
  // std::cout << "target speed: " << targetPoint_.speed << std::endl;
  if (std::abs(commandSpeed) > targetPoint_.speed()) {
    double timeToSpeed = std::abs((robotLSpeed - targetPoint_.speed()) / MAX_LINEAR_ACCELERATION);
    // Factor is needed, else the length is largely underestimated. Probably because of control rate?
    double lengthToSpeed = LINEAR_CONTROL_STOP_DISTANCE_FACTOR * (robotLSpeed * timeToSpeed - 0.5 * acceleration * timeToSpeed * timeToSpeed);
    Point robot2SpeedPoint(lengthToSpeed * robotPose.theta().cos(),
                           lengthToSpeed * robotPose.theta().sin());  // TODO(guilhembn): This may not translate for holonomic robot
    Point speedPoint = robot2SpeedPoint + robotPose;
    // Ivy::getInstance().sendPoint(1, speedPoint);
    Point pSpeed = speedPoint - targetPoint_;
    double plannedSpeedError = pSpeed.norm();
    if (plannedSpeedError <= ADMITTED_LINEAR_POSITION_ERROR || pSpeed.dot(-rp) < 0) {
      // We will get at the target point with the right speed, or overshoot it. We need to decelerate
      double speedDiff = acceleration * dt;
      if (std::abs(speedDiff) >= std::abs(robotLSpeed)) {
        commandSpeed = 0.;
      } else {
        commandSpeed = robotLSpeed - speedDiff;
      }
    } else if (plannedSpeedError <= 3 * ADMITTED_LINEAR_POSITION_ERROR && std::abs(robotLSpeed) > 10.) {
      commandSpeed = robotLSpeed;
    }
  }
  commandSpeed = std::min(MAX_LINEAR_SPEED, std::max(-MAX_LINEAR_SPEED, commandSpeed));
  if (IS_HOLONOMIC) {
    Point robot2target = targetPoint_.transformIn(robotPose);
    Angle robot2targetAngle = robot2target.polarAngle();
    return Speed(commandSpeed * robot2targetAngle.cos(), commandSpeed * robot2targetAngle.sin(), 0.0);
  } else {
    return Speed(commandSpeed, 0.0, 0.0);  // We assume the direction will be handled via steering
  }
}
}  // namespace rd