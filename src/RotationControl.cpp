#include "Navigation/RotationControl.h"

#include "Navigation/Parameters.h"

namespace rd {

RotationControl::RotationControl(const PositionControlParameters &params) : PositionControlBase(params) {}

Speed RotationControl::computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed, double dt) {
  Angle diff = targetAngle_ - robotPose.theta();
  double acceleration;
  if (diff < 0) {
    acceleration = -params_.maxRotationalAcceleration;
  } else {
    acceleration = params_.maxRotationalAcceleration;
  }
  double durationToStop = robotSpeed.vtheta() / acceleration;

  Angle plannedStopAngle = robotPose.theta() + Angle(2 * (robotSpeed.vtheta() * durationToStop - 0.5 * acceleration * durationToStop * durationToStop));
  Angle plannedError = targetAngle_ - plannedStopAngle;
  double speedCommand;
  if (std::abs(plannedError.value()) <= params_.admittedAnglePositionError ||
      diff.value() * plannedError.value() < 0.0) {  // If the next error is smaller than the acceptable error, or if we will overshoot the target angle
    double speedDiff = acceleration * dt;
    if (std::abs(speedDiff) > std::abs(robotSpeed.vtheta())) {  // The acceleration to stop is less than max acceleration, so stop
      speedCommand = 0.0;
    } else {
      speedCommand = robotSpeed.vtheta() - speedDiff;  // Apply max acceleration to brake
    }
  } else if (std::abs(plannedError.value()) <= 3. * params_.admittedAnglePositionError && std::abs(robotSpeed.vtheta()) >= 0.3) {
    // If we next loop will not be too far from target, and the robot is moving at an acceptable speed, do not brake, do not accelerate
    speedCommand = robotSpeed.vtheta();
  } else {
    // We still have path to travel
    double mul = 1.;
    if (std::abs(diff.value()) <= 3. * params_.admittedAnglePositionError) {
      mul = 0.7;
    }
    speedCommand = robotSpeed.vtheta() + dt * acceleration * mul;
  }
  speedCommand = std::min(params_.maxRotationalSpeed, std::max(-params_.maxRotationalSpeed, speedCommand));
  return Speed(0.0, 0.0, speedCommand);
}
}  // namespace rd
