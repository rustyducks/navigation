#ifndef ROTATIONCONTROL_H
#define ROTATIONCONTROL_H

#include "GeometryTools/Point.h"
#include "Navigation/PositionControlBase.h"
namespace rd {
class RotationControl : public PositionControlBase {
 public:
  RotationControl(const PositionControlParameters &params);
  ~RotationControl() = default;

  virtual Speed computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed, double dt) override;
  void setTargetAngle(const Angle &angle);  // In map frame (absolute)
  bool isGoalReached() const override { return isGoalReached_; }

 protected:
  enum eRotationControlState { ACCELERATE, CRUISING, DECELERATE };
  eRotationControlState state_;
  bool isGoalReached_;

  Angle targetAngle_;  // In map frame (absolute)
};

}  // namespace rd

#endif /* ROTATIONCONTROL_H */
