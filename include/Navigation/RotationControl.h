#ifndef ROTATIONCONTROL_H
#define ROTATIONCONTROL_H

#include "Navigation/Geometry/Point.h"
#include "Navigation/PositionControlBase.h"
namespace rd {
class RotationControl : public PositionControlBase {
public:
  RotationControl();
  ~RotationControl() = default;

  virtual Speed computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed) override;
  void setTargetAngle(const Angle &angle) { targetAngle_ = angle; }; // In map frame (absolute)

protected:
  Angle targetAngle_; // In map frame (absolute)
};

} // namespace rd

#endif /* ROTATIONCONTROL_H */
