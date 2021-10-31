#ifndef LINEARCONTROL_H
#define LINEARCONTROL_H

#include "GeometryTools/Trajectory.h"
#include "Navigation/PositionControlBase.h"

namespace rd {
class LinearControl : public PositionControlBase {
 public:
  LinearControl();
  ~LinearControl() = default;
  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) override;
  inline void setTargetPoint(const PointOrientedSpeed& target) { targetPoint_ = target; };

 protected:
  PointOrientedSpeed targetPoint_;
};
}  // namespace rd

#endif /* LINEARCONTROL_H */
