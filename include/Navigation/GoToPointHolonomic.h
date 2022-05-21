#ifndef GOTOPOINTHOLONOMIC_H
#define GOTOPOINTHOLONOMIC_H

#include "Navigation/PositionControlBase.h"

namespace rd {
class GoToPointHolonomic : public PositionControlBase {
 public:
  GoToPointHolonomic(const PositionControlParameters& params, int stopDistanceFactor = LINEAR_CONTROL_STOP_DISTANCE_FACTOR);
  ~GoToPointHolonomic() = default;
  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt, double maxSpeed) override;
  inline void setTargetPoint(const PointOriented& target) {
    targetPoint_ = target;
    isGoalReached_ = false;
    linearState_ = eLinearState::LACCELERATE;
    rotationState_ = eRotationState::RACCELERATE;
  };
  bool isGoalReached() const override { return isGoalReached_; }
  void slowedDown() {
    if (!isGoalReached_) {
      linearState_ = LACCELERATE;
    }
  }

 protected:
  enum eLinearState { LIDLE, LACCELERATE, LCRUISE, LDECELERATE, LMAINTAIN };
  enum eRotationState { RIDLE, RACCELERATE, RCRUISE, RDECELERATE, RMAINTAIN };
  eLinearState linearState_;
  eRotationState rotationState_;
  PointOriented targetPoint_;
  const int stopDistanceFactor_;
  bool isGoalReached_;
};
}  // namespace rd

#endif /* GOTOPOINTHOLONOMIC_H */
