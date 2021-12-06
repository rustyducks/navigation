#include "Navigation/RotationControl.h"

#include "Navigation/Parameters.h"

namespace rd {

RotationControl::RotationControl(const PositionControlParameters &params) : PositionControlBase(params), state_(eRotationControlState::ACCELERATE) {}

Speed RotationControl::computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed, double dt) {
  double rotationSpeed = 0.;
  Angle diff = targetAngle_ - robotPose.theta();
  double angleToStop;
  if (state_ == eRotationControlState::ACCELERATE) {
    if (diff < 0) {
      rotationSpeed = robotSpeed.vtheta() - params_.maxRotationalAcceleration;
    } else {
      rotationSpeed = robotSpeed.vtheta() + params_.maxRotationalAcceleration;
    }
    if (std::abs(rotationSpeed) >= params_.maxRotationalSpeed) {
      state_ = eRotationControlState::CRUISING;
      rotationSpeed = std::min(params_.maxRotationalSpeed, std::max(-params_.maxRotationalSpeed, rotationSpeed));
    }
    angleToStop = 0.5 * robotSpeed.vtheta() * robotSpeed.vtheta() / params_.maxRotationalAcceleration;
    if (angleToStop + dt * robotSpeed.vtheta() >= std::abs(diff.value() + rotationSpeed * dt)) {
      state_ = eRotationControlState::DECELERATE;
    }
  }
  if (state_ == eRotationControlState::CRUISING) {
    if (diff < 0) {
      rotationSpeed = -params_.maxRotationalSpeed;
    } else {
      rotationSpeed = params_.maxRotationalSpeed;
    }
    angleToStop = 0.5 * robotSpeed.vtheta() * robotSpeed.vtheta() / params_.maxRotationalAcceleration;
    if (angleToStop + dt * robotSpeed.vtheta() >= std::abs(diff.value() + rotationSpeed * dt)) {
      state_ = eRotationControlState::DECELERATE;
    }
  }
  if (state_ == eRotationControlState::DECELERATE) {
    if (diff < 0) {
      rotationSpeed = std::min(-params_.minRotationalSpeed, robotSpeed.vtheta() + params_.maxRotationalAcceleration);
    } else {
      rotationSpeed = std::max(params_.minRotationalSpeed, robotSpeed.vtheta() - params_.maxRotationalAcceleration);
    }
    if (std::abs(diff.value()) <= params_.admittedAnglePositionError) {
      rotationSpeed = 0.;
      isGoalReached_ = true;
    }
  }
  return Speed(0.0, 0.0, rotationSpeed);
}

void RotationControl::setTargetAngle(const Angle &angle) {
  targetAngle_ = angle;
  state_ = eRotationControlState::ACCELERATE;
  isGoalReached_ = false;
}
}  // namespace rd
