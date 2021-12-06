#ifndef PUREPURSUITCONTROL_H
#define PUREPURSUITCONTROL_H

#include "GeometryTools/Trajectory.h"
#include "Navigation/LinearControl.h"
#include "Navigation/PositionControlBase.h"
#include "Navigation/RotationControl.h"

namespace rd {
class PurePursuitControl : public PositionControlBase {
 public:
  PurePursuitControl(const PositionControlParameters& params, int linearControlStopDistanceFactor = LINEAR_CONTROL_STOP_DISTANCE_FACTOR,
                     double lookaheadDistance = PURE_PURSUIT_LOOKAHEAD_DISTANCE);
  ~PurePursuitControl() = default;

  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) override;
  void setTrajectory(const Trajectory& trajectory) {
    trajectory_ = trajectory;
    trajectoryCurrentIndex_ = 1;
    state_ = FIRST_ROTATION;
    rotationAngleToSet_ = true;
    isGoalReached_ = false;
  };
  const Trajectory& getTrajectory() const { return trajectory_; }
  bool isGoalReached() const override { return isGoalReached_; }

 protected:
  enum PurePursuitState { IDLE, FIRST_ROTATION, ACCELERATE, CRUISING, DECELERATE, LAST_ROTATION };

  Speed cruising(const PointOriented& robotPose, const Speed& robotSpeed, double dt);
  Speed purePursuit(const PointOriented& robotPose, const double linearSpeed, double dt);

  PurePursuitState state_;
  Trajectory trajectory_;
  RotationControl rotationControl_;
  LinearControl linearControl_;
  size_t trajectoryCurrentIndex_;
  bool isGoalReached_;
  bool rotationAngleToSet_;

  const double lookaheadDistance_;
};

}  // namespace rd

#endif /* PUREPURSUITCONTROL_H */
