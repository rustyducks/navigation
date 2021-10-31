#ifndef PUREPURSUITCONTROL_H
#define PUREPURSUITCONTROL_H

#include "GeometryTools/Trajectory.h"
#include "Navigation/LinearControl.h"
#include "Navigation/PositionControlBase.h"
#include "Navigation/RotationControl.h"

namespace rd {
class PurePursuitControl : public PositionControlBase {
 public:
  PurePursuitControl();
  ~PurePursuitControl() = default;

  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) override;
  void setTrajectory(const Trajectory& trajectory) {
    trajectory_ = trajectory;
    trajectoryCurrentIndex_ = 1;
    state_ = FIRST_ROTATION;
    isGoalReached_ = false;
  };
  const Trajectory& getTrajectory() const { return trajectory_; }
  bool isGoalReached() const { return isGoalReached_; }

 protected:
  enum PurePursuitState { IDLE, FIRST_ROTATION, CRUISING, LAST_ROTATION };

  Speed cruising(const PointOriented& robotPose, const Speed& robotSpeed, double dt);
  Speed purePursuit(const PointOriented& robotPose, const Speed& robotSpeed, double dt);

  PurePursuitState state_;
  Trajectory trajectory_;
  RotationControl rotationControl_;
  LinearControl linearControl_;
  size_t trajectoryCurrentIndex_;
  bool isGoalReached_;
};

}  // namespace rd

#endif /* PUREPURSUITCONTROL_H */
