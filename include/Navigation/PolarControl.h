#ifndef POLARCONTROL_H
#define POLARCONTROL_H

#include "Navigation/PositionControlBase.h"

namespace rd {
class PolarControl : public PositionControlBase {
 public:
  PolarControl(const PositionControlParameters& params, int stopDistanceFactor = LINEAR_CONTROL_STOP_DISTANCE_FACTOR);
  ~PolarControl() = default;

  const double pangle = 2.;
  const double iangle = 0.0;
  const double dangle = 0.0;

  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt, double maxSpeed) override;
  inline void setTargetPoint(const PointOriented& target, bool backwards, bool withFirstRotation) {
    targetPoint_ = target;
    isGoalReached_ = false;
    angleIntegral = 0.;
    previousAngleError = 0.;
    backwards_ = backwards;
    if (withFirstRotation) {
      state_ = eState::FIRST_ROTATION;
    } else {
      state_ = eState::ACCELERATE;
    }
  };
  bool isGoalReached() const override { return isGoalReached_; }
  void slowedDown() {
    if (!isGoalReached_) {
      state_ = ACCELERATE;
    }
  }

 protected:
  enum eState { IDLE, FIRST_ROTATION, ACCELERATE, CRUISE, DECELERATE, LAST_ROTATION };
  eState state_;
  PointOriented targetPoint_;
  const int stopDistanceFactor_;
  bool isGoalReached_;
  double angleIntegral;
  double previousAngleError;
  bool backwards_;
};
}  // namespace rd

#endif /* POLARCONTROL_H */
