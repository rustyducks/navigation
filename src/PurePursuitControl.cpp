#include "Navigation/PurePursuitControl.h"

// #include "Navigation/Communication/Ivy.h"
#include "Navigation/Parameters.h"

namespace rd {
PurePursuitControl::PurePursuitControl(const PositionControlParameters& params, int linearControlStopDistanceFactor, double lookaheadDistance)
    : PositionControlBase(params),
      trajectory_(),
      rotationControl_(params),
      linearControl_(params, linearControlStopDistanceFactor),
      isGoalReached_(false),
      rotationAngleToSet_(false),
      lookaheadDistance_(lookaheadDistance) {}

Speed PurePursuitControl::computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt, double maxSpeed) {
  Angle targetAngle;
  switch (state_) {
    case PurePursuitState::IDLE:
      return Speed(0., 0., 0.);
      break;
    case PurePursuitState::FIRST_ROTATION:
      if (rotationAngleToSet_) {
        targetAngle = robotPose.angleTo(trajectory_.at(1));  // trajectory.at(0) is past point, in order for the interpolation to work
        rotationControl_.setTargetAngle(targetAngle);
        rotationAngleToSet_ = false;
      }
      if (std::abs(robotSpeed.vtheta()) <= params_.maxRotationalAcceleration && rotationControl_.isGoalReached()) {
        state_ = PurePursuitState::ACCELERATE;
        return cruising(robotPose, robotSpeed, dt, maxSpeed);
      } else {
        return rotationControl_.computeSpeed(robotPose, robotSpeed, dt, maxSpeed);
      }
      break;
    case PurePursuitState::ACCELERATE:
    case PurePursuitState::CRUISING:
    case PurePursuitState::DECELERATE:
      return cruising(robotPose, robotSpeed, dt, maxSpeed);
      break;
    case PurePursuitState::LAST_ROTATION:
      if (std::abs(robotSpeed.vtheta()) <= 0.01 && std::abs((trajectory_.at(0).theta() - robotPose.theta()).value()) < params_.admittedAnglePositionError) {
        trajectory_.pop();
        assert(trajectory_.size() == 0);
        state_ = PurePursuitState::IDLE;
        isGoalReached_ = true;
        return Speed(0.0, 0.0, 0.0);
      } else {
        return rotationControl_.computeSpeed(robotPose, robotSpeed, dt, params_.maxRotationalSpeed);
      }
      break;
    default:
      throw std::runtime_error("Invalid state in PurePursuitController.");
  }
  return Speed(0.0, 0.0, 0.0);
}

Speed PurePursuitControl::cruising(const PointOriented& robotPose, const Speed& robotSpeed, double dt, double maxSpeed) {
  double vx = robotSpeed.vx();
  double lengthToStop;
  if (state_ == PurePursuitState::ACCELERATE) {
    vx = robotSpeed.vx() + params_.maxLinearAcceleration * dt;
    if (vx >= maxSpeed) {
      vx = maxSpeed;
      state_ = PurePursuitControl::ACCELERATE;
    } else if (vx >= params_.maxLinearSpeed) {
      state_ = PurePursuitState::CRUISING;
      vx = params_.maxLinearSpeed;
    }
    lengthToStop = 0.5 * robotSpeed.vx() * robotSpeed.vx() / params_.maxLinearAcceleration;
    if (lengthToStop + dt * vx >= trajectory_.distanceBetween(robotPose, trajectory_.last())) {
      state_ = PurePursuitState::DECELERATE;
    }
  }
  if (state_ == PurePursuitState::CRUISING) {
    vx = params_.maxLinearSpeed;
    if (vx >= maxSpeed) {
      vx = maxSpeed;
      state_ = PurePursuitControl::ACCELERATE;
    }
    lengthToStop = 0.5 * robotSpeed.vx() * robotSpeed.vx() / params_.maxLinearAcceleration;
    if (lengthToStop + dt * robotSpeed.vx() >= trajectory_.distanceBetween(robotPose, trajectory_.last())) {
      state_ = PurePursuitState::DECELERATE;
    }
  }
  if (state_ == PurePursuitState::DECELERATE) {
    vx = std::max(params_.minLinearSpeed, vx - params_.maxLinearAcceleration * dt);
    if (vx >= maxSpeed) {
      vx = maxSpeed;
      state_ = PurePursuitControl::ACCELERATE;
    }
  }

  PointOriented nextTrajPoint = trajectory_.at(trajectoryCurrentIndex_);  // trajectory.at(0) is past point, in order for the interpolation to work
  // Ivy::getInstance().sendPoint(5, nextTrajPoint);

  // double nextTrajSpeed = trajectory_.at(trajectoryCurrentIndex_).speed();
  double positionError = robotPose.distanceTo(nextTrajPoint);
  if (positionError < params_.admittedLinearPositionError) {
    // If we are not too far
    if (trajectoryCurrentIndex_ == trajectory_.size() - 1) {
      // If it is the last point of the trajectory, start final rotation
      for (size_t i = 0; i < trajectoryCurrentIndex_; i++) {
        trajectory_.pop();
      }
      trajectoryCurrentIndex_ = 1;
      state_ = PurePursuitState::LAST_ROTATION;
      rotationControl_.setTargetAngle(nextTrajPoint.theta());
      return rotationControl_.computeSpeed(robotPose, robotSpeed, dt, params_.maxRotationalSpeed);
      //} else if (nextTrajSpeed < 0.1) {
      // If it is not the last point, but we should stop, start a new initial rotation
      //  for (size_t i = 0; i < trajectoryCurrentIndex_; i++) {
      //    trajectory_.pop();
      //  }
      //  trajectoryCurrentIndex_ = 1;
      //  state_ = PurePursuitState::FIRST_ROTATION;
      //  Angle targetAngle = robotPose.angleTo(trajectory_.at(1));
      //  rotationControl_.setTargetAngle(targetAngle);
      //  return rotationControl_.computeSpeed(robotPose, robotSpeed, dt);
    } else {
      // Go to the next point in cruising
      for (size_t i = 0; i < trajectoryCurrentIndex_; i++) {
        trajectory_.pop();
      }
      trajectoryCurrentIndex_ = 1;
      return purePursuit(robotPose, vx);
    }
  } else {
    return purePursuit(robotPose, vx);
  }
}

Speed PurePursuitControl::purePursuit(const PointOriented& robotPose, const double linearSpeed) {
  // Ivy& ivy = Ivy::getInstance();
  size_t nextClosestIndex;
  trajectory_.pointWithSpeedAtDistanceFrom(0, robotPose, nextClosestIndex);
  // linearControl_.setTargetPoint(trajectory_.at(nextClosestIndex + 1));
  //  Speed linear = linearControl_.computeSpeed(robotPose, robotSpeed, dt);
  //   std::cout << "linear Speed: " << linear << std::endl;

  // double vx = linear.linearSpeed();  // Only the magnitude of the speed interest us, the rest is handled via steering
  //  double vx = 100;
  for (size_t i = 0; i < nextClosestIndex; i++) {
    trajectory_.pop();
  }
  double vx = linearSpeed;
  size_t previousClosestIndex;
  Point goal = trajectory_.pointWithSpeedAtDistanceFrom(lookaheadDistance_, robotPose,
                                                        previousClosestIndex);  // If goal is last point (or speed == 0) go into polar control
  trajectoryCurrentIndex_ = previousClosestIndex + 1;
  // ivy.sendPoint(1, goal);
  Point robot2Goal = goal.transformIn(robotPose);
  double curvature = 2 * robot2Goal.y() / robot2Goal.squaredNorm();
  double vtheta = vx * curvature;
  /*if (std::abs(vtheta) > params_.maxRotationalSpeed) {
    vtheta = std::min(params_.maxRotationalSpeed, std::max(-params_.maxRotationalSpeed, vtheta));
    // vtheta = std::min(robotSpeed.vtheta() + param<double>(MAX_ROTATIONAL_ACCELERATION) * dt,
    //                  std::max(robotSpeed.vtheta() - param<double>(MAX_ROTATIONAL_ACCELERATION) * dt, vtheta));
    assert(curvature != 0.0);  // curvature can't be 0, else, vtheta would have been 0, and not exceeding max speed.
    vx = vtheta / curvature;
  }*/
  return Speed(vx, 0.0, vtheta);
}
}  // namespace rd
