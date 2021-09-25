#include "Navigation/PurePursuitControl.h"

namespace rd {
PurePursuitControl::PurePursuitControl() : trajectory_() {}

Speed PurePursuitControl::computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed) {
  Angle targetAngle;
  switch (state_) {
  case PurePursuitState::IDLE:
    return Speed(0, 0, 0);
    break;
  case PurePursuitState::FIRST_ROTATION:

    targetAngle = robotPose.angleTo(trajectory_.at(0).point);
    if (std::abs(robotSpeed.vtheta()) <= 0.01 && std::abs((targetAngle - robotPose.theta()).value()) < 0.05) { // Todo (guilhembn): Find a way to pass parameters
      state_ = PurePursuitState::CRUISING;
    } else {
      rotationControl_.setTargetAngle(targetAngle);
      return rotationControl_.computeSpeed(robotPose, robotSpeed);
    }
    break;

  case PurePursuitState::CRUISING:
    return Speed(0.0, 0.0, 0.0);
    break;
  case PurePursuitState::LAST_ROTATION:
    if (std::abs(robotSpeed.vtheta()) <= 0.01 && std::abs((targetAngle - robotPose.theta()).value()) < 0.05) {
      trajectory_.pop();
      assert(trajectory_.size() == 0);
      state_ = PurePursuitState::IDLE;
      return Speed(0.0, 0.0, 0.0);
    } else {
      return rotationControl_.computeSpeed(robotPose, robotSpeed);
    }
    break;
  default:
    throw std::runtime_error("Invalid state in PurePursuitController.");
  }
  return Speed(0.0, 0.0, 0.0);
}
} // namespace rd
