#ifndef PUREPURSUITCONTROL_H
#define PUREPURSUITCONTROL_H

#include "Navigation/Geometry/Trajectory.h"
#include "Navigation/PositionControlBase.h"
#include "Navigation/RotationControl.h"

namespace rd {
class PurePursuitControl : public PositionControlBase {
 public:
  PurePursuitControl();
  ~PurePursuitControl() = default;

  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) override;
  void setTrajectory(const Trajectory& trajectory) { trajectory_ = trajectory; };
  const Trajectory& getTrajectory() const { return trajectory_; }

 protected:
  enum PurePursuitState { IDLE, FIRST_ROTATION, CRUISING, LAST_ROTATION };
  PurePursuitState state_;
  Trajectory trajectory_;
  RotationControl rotationControl_;
};

}  // namespace rd

#endif /* PUREPURSUITCONTROL_H */
