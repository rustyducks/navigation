#include "Navigation/RotationControl.h"

#include "Navigation/Parameters.h"

namespace rd {

RotationControl::RotationControl() {}

Speed RotationControl::computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed, double dt) {
  Angle diff = targetAngle_ - robotPose.theta();
  double acceleration;
  if (diff < 0) {
    acceleration = -MAX_ROTATIONAL_ACCELERATION;
  } else {
    acceleration = MAX_ROTATIONAL_ACCELERATION;
  }
  double durationToStop = robotSpeed.vtheta() / acceleration;

  Angle plannedStopAngle = robotPose.theta() + Angle(2 * (robotSpeed.vtheta() * durationToStop - 0.5 * acceleration * durationToStop * durationToStop));
  Angle plannedError = targetAngle_ - plannedStopAngle;
  double speedCommand;
  if (std::abs(plannedError.value()) <= ADMITTED_ANGLE_POSITION_ERROR ||
      diff.value() * plannedError.value() < 0.0) {  // If the next error is smaller than the acceptable error, or if we will overshoot the target angle
    double speedDiff = acceleration * dt;
    if (std::abs(speedDiff) > std::abs(robotSpeed.vtheta())) {  // The acceleration to stop is less than max acceleration, so stop
      speedCommand = 0.0;
    } else {
      speedCommand = robotSpeed.vtheta() - speedDiff;  // Apply max acceleration to brake
    }
  } else if (std::abs(plannedError.value()) <= 3. * ADMITTED_ANGLE_POSITION_ERROR && std::abs(robotSpeed.vtheta()) >= 0.3) {
    // If we next loop will not be too far from target, and the robot is moving at an acceptable speed, do not brake, do not accelerate
    speedCommand = robotSpeed.vtheta();
  } else {
    // We still have path to travel
    double mul = 1.;
    if (std::abs(diff.value()) <= 3. * ADMITTED_ANGLE_POSITION_ERROR) {
      mul = 0.7;
    }
    speedCommand = robotSpeed.vtheta() + dt * acceleration * mul;
  }
  speedCommand = std::min(MAX_ROTATIONAL_SPEED, std::max(-MAX_ROTATIONAL_SPEED, speedCommand));
  return Speed(0.0, 0.0, speedCommand);
}
}  // namespace rd
