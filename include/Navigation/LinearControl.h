#ifndef LINEARCONTROL_H
#define LINEARCONTROL_H

#include "GeometryTools/Trajectory.h"
#include "Navigation/PositionControlBase.h"

namespace rd {
class LinearControl : public PositionControlBase {
 public:
  LinearControl(const PositionControlParameters& params, int stopDistanceFactor = LINEAR_CONTROL_STOP_DISTANCE_FACTOR);
  ~LinearControl() = default;
  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt, double maxSpeed) override;
  inline void setTargetPoint(const PointOrientedSpeed& target) { targetPoint_ = target; };
  bool isGoalReached() const override { throw std::runtime_error("Not implemented"); }

 protected:
  PointOrientedSpeed targetPoint_;
  const int stopDistanceFactor_;
};
}  // namespace rd

#endif /* LINEARCONTROL_H */
