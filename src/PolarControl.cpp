#include "Navigation/PolarControl.h"

namespace rd {

PolarControl::PolarControl(const PositionControlParameters& params, int stopDistanceFactor)
    : PositionControlBase(params), state_(eState::IDLE), stopDistanceFactor_(stopDistanceFactor), isGoalReached_(true) {}

Speed PolarControl::computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt, double maxSpeed) {
  Point rp = targetPoint_ - robotPose;
  double vlinear = robotSpeed.linearSpeed();
  double lengthToStop;
  if (state_ == eState::IDLE) {
    vlinear = 0.;
    return Speed(0., 0., 0.);
  }
  if (state_ == eState::FIRST_ROTATION) {
    vlinear = 0.;
  }
  if (state_ == eState::ACCELERATE) {
    vlinear = robotSpeed.linearSpeed() + params_.maxLinearAcceleration * dt;
    if (vlinear >= maxSpeed) {
      vlinear = maxSpeed;
      state_ = eState::ACCELERATE;
    } else if (vlinear >= params_.maxLinearSpeed) {
      state_ = eState::CRUISE;
      vlinear = params_.maxLinearSpeed;
    }
    lengthToStop = 0.5 * robotSpeed.linearSpeed() * robotSpeed.linearSpeed() / params_.maxLinearAcceleration * stopDistanceFactor_;
    if (lengthToStop + dt * vlinear >= rp.norm()) {
      state_ = eState::DECELERATE;
    }
  }
  if (state_ == eState::CRUISE) {
    vlinear = params_.maxLinearSpeed;
    if (vlinear >= maxSpeed) {
      vlinear = maxSpeed;
      state_ = eState::ACCELERATE;
    }
    lengthToStop = 0.5 * robotSpeed.linearSpeed() * robotSpeed.linearSpeed() / params_.maxLinearAcceleration * stopDistanceFactor_;
    if (lengthToStop + dt * robotSpeed.linearSpeed() >= rp.norm()) {
      state_ = eState::DECELERATE;
    }
  }
  if (state_ == eState::DECELERATE) {
    vlinear = std::max(params_.minLinearSpeed, vlinear - params_.maxLinearAcceleration * dt);
    if (vlinear >= maxSpeed) {
      vlinear = maxSpeed;
      state_ = eState::ACCELERATE;
    }
  }
  double positionError = rp.norm();

  if (state_ == eState::LAST_ROTATION) {
    vlinear = -robotSpeed.vx();
  } else if (positionError < params_.admittedLinearPositionError ||
             (positionError < 5 * params_.admittedLinearPositionError && robotSpeed.vx() * rp.x() < 0.)) {
    state_ = eState::LAST_ROTATION;
    angleIntegral = 0.;
    previousAngleError = 0.;
    vlinear = 0;
  }

  Angle diff = Angle(0.);
  double vtheta = 0;
  if (state_ == eState::ACCELERATE || state_ == eState::CRUISE || state_ == eState::DECELERATE) {
    Point robot2target = targetPoint_.transformIn(robotPose);
    diff = robot2target.polarAngle();
    if (backwards_) {
      diff += Angle(M_PI);
    }
    angleIntegral += diff.value();
    angleIntegral = std::min(3 * params_.maxRotationalSpeed, std::max(-3 * params_.maxRotationalSpeed, angleIntegral));
    double derivative = (diff.value() - previousAngleError) / dt;
    previousAngleError = diff.value();
    vtheta = diff.value() * pangle + iangle * angleIntegral + dangle * derivative;
    if (rp.norm() < 40.0) {
      vtheta /= 10.;
    }

  } else if (state_ == eState::LAST_ROTATION) {
    diff = targetPoint_.theta() - robotPose.theta();
    double derivative = (diff.value() - previousAngleError) / dt;

    vtheta = diff.value() * pangle + iangle * angleIntegral + dangle * derivative;

    if (std::abs(diff.value()) < params_.admittedAnglePositionError) {
      state_ = IDLE;
      isGoalReached_ = true;
      return Speed(0., 0., 0.);
    }
  } else if (state_ == eState::FIRST_ROTATION) {
    Point robot2target = targetPoint_.transformIn(robotPose);
    diff = robot2target.polarAngle();
    if (backwards_) {
      diff += Angle(M_PI);
    }
    angleIntegral += diff.value();
    angleIntegral = std::min(3 * params_.maxRotationalSpeed, std::max(-3 * params_.maxRotationalSpeed, angleIntegral));
    double derivative = (diff.value() - previousAngleError) / dt;
    previousAngleError = diff.value();
    vtheta = diff.value() * pangle + iangle * angleIntegral + dangle * derivative;
    if (std::abs(diff.value()) < params_.admittedAnglePositionError) {
      state_ = ACCELERATE;
      return Speed(0., 0., 0.);
    }
  }
  // std::cout << robot2target << std::endl;
  vtheta = std::max(-params_.maxRotationalSpeed, std::min(params_.maxRotationalSpeed, vtheta));
  if (std::abs(vlinear) < 0.001) {
    if (std::abs(vtheta) > 0.001) {
      vtheta = vtheta > 0 ? std::max(params_.minRotationalSpeed, vtheta) : std::min(-params_.minRotationalSpeed, vtheta);
    } else {
      vtheta = 0.;
    }
  }

  if (backwards_) {
    vlinear = -vlinear;
  }

  return Speed(vlinear, 0., vtheta);
}
}  // namespace rd
