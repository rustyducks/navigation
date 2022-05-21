#include "Navigation/GoToPointHolonomic.h"

namespace rd {

GoToPointHolonomic::GoToPointHolonomic(const PositionControlParameters& params, int stopDistanceFactor)
    : PositionControlBase(params),
      linearState_(eLinearState::LIDLE),
      rotationState_(eRotationState::RIDLE),
      stopDistanceFactor_(stopDistanceFactor),
      isGoalReached_(true) {}

Speed GoToPointHolonomic::computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt, double maxSpeed) {
  Point rp = targetPoint_ - robotPose;
  double vlinear = robotSpeed.linearSpeed();
  double lengthToStop;
  if (linearState_ == eLinearState::LIDLE) {
    vlinear = 0.;
  }
  if (linearState_ == eLinearState::LACCELERATE) {
    vlinear = robotSpeed.linearSpeed() + params_.maxLinearAcceleration * dt;
    if (vlinear >= maxSpeed) {
      vlinear = maxSpeed;
      linearState_ = eLinearState::LACCELERATE;
    } else if (vlinear >= params_.maxLinearSpeed) {
      linearState_ = eLinearState::LCRUISE;
      vlinear = params_.maxLinearSpeed;
    }
    lengthToStop = 0.5 * robotSpeed.linearSpeed() * robotSpeed.linearSpeed() / params_.maxLinearAcceleration * stopDistanceFactor_;
    if (lengthToStop + dt * vlinear >= rp.norm()) {
      linearState_ = eLinearState::LDECELERATE;
    }
  }
  if (linearState_ == eLinearState::LCRUISE) {
    vlinear = params_.maxLinearSpeed;
    if (vlinear >= maxSpeed) {
      vlinear = maxSpeed;
      linearState_ = eLinearState::LACCELERATE;
    }
    lengthToStop = 0.5 * robotSpeed.linearSpeed() * robotSpeed.linearSpeed() / params_.maxLinearAcceleration * stopDistanceFactor_;
    if (lengthToStop + dt * robotSpeed.linearSpeed() >= rp.norm()) {
      linearState_ = eLinearState::LDECELERATE;
    }
  }
  if (linearState_ == eLinearState::LDECELERATE) {
    vlinear = std::max(params_.minLinearSpeed, vlinear - params_.maxLinearAcceleration * dt);
    if (vlinear >= maxSpeed) {
      vlinear = maxSpeed;
      linearState_ = eLinearState::LACCELERATE;
    }
  }
  double positionError = rp.norm();

  if (linearState_ == eLinearState::LMAINTAIN) {
    if (positionError > params_.admittedLinearPositionError) {
      vlinear = params_.minLinearSpeed;
    } else {
      vlinear = 0.;
    }
  } else if (positionError < params_.admittedLinearPositionError) {
    linearState_ = eLinearState::LMAINTAIN;
    vlinear = 0.;
  }

  double rotationSpeed = 0.;
  Angle diff = targetPoint_.theta() - robotPose.theta();
  double angleToStop;
  if (rotationState_ == eRotationState::RIDLE) {
    rotationSpeed = 0.;
  }
  if (rotationState_ == eRotationState::RACCELERATE) {
    if (diff < 0) {
      rotationSpeed = robotSpeed.vtheta() - params_.maxRotationalAcceleration * dt;
    } else {
      rotationSpeed = robotSpeed.vtheta() + params_.maxRotationalAcceleration * dt;
    }
    if (std::abs(rotationSpeed) >= params_.maxRotationalSpeed) {
      rotationState_ = eRotationState::RCRUISE;
      rotationSpeed = std::min(params_.maxRotationalSpeed, std::max(-params_.maxRotationalSpeed, rotationSpeed));
    }
    angleToStop = 0.5 * robotSpeed.vtheta() * robotSpeed.vtheta() / params_.maxRotationalAcceleration * stopDistanceFactor_;
    if (angleToStop + dt * std::abs(rotationSpeed) >= std::abs(diff.value())) {
      rotationState_ = eRotationState::RDECELERATE;
    }
  }
  if (rotationState_ == eRotationState::RCRUISE) {
    if (diff < 0) {
      rotationSpeed = -params_.maxRotationalSpeed;
    } else {
      rotationSpeed = params_.maxRotationalSpeed;
    }
    angleToStop = 0.5 * robotSpeed.vtheta() * robotSpeed.vtheta() / params_.maxRotationalAcceleration * stopDistanceFactor_;
    if (angleToStop + dt * std::abs(robotSpeed.vtheta()) >= std::abs(diff.value())) {
      rotationState_ = eRotationState::RDECELERATE;
    }
  }
  if (rotationState_ == eRotationState::RDECELERATE) {
    if (diff < 0) {
      rotationSpeed = std::min(-params_.minRotationalSpeed, robotSpeed.vtheta() + params_.maxRotationalAcceleration * dt);
    } else {
      rotationSpeed = std::max(params_.minRotationalSpeed, robotSpeed.vtheta() - params_.maxRotationalAcceleration * dt);
    }
    if (std::abs(diff.value()) <= params_.admittedAnglePositionError) {
      rotationSpeed = 0.;
      rotationState_ = eRotationState::RMAINTAIN;
    }
  }
  if (rotationState_ == eRotationState::RMAINTAIN) {
    if (std::abs(diff.value()) > params_.admittedAnglePositionError) {
      if (diff < 0) {
        rotationSpeed = -params_.minRotationalSpeed;
      } else {
        rotationSpeed = params_.maxRotationalSpeed;
      }
    } else {
      rotationSpeed = 0.;
    }
  }

  if (rotationState_ == eRotationState::RMAINTAIN && linearState_ == eLinearState::LMAINTAIN) {
    isGoalReached_ = true;
    rotationState_ = eRotationState::RIDLE;
    linearState_ = eLinearState::LIDLE;
  }

  rd::Point robot2point = targetPoint_.transformIn(robotPose);
  Angle angleToTarget = robot2point.polarAngle() + Angle(rotationSpeed * dt);

  return Speed(vlinear * angleToTarget.cos(), vlinear * angleToTarget.sin(), rotationSpeed);
}
}  // namespace rd
