#ifndef POSITIONCONTROLBASE_H
#define POSITIONCONTROLBASE_H

#include <string>
#include <unordered_map>

#include "GeometryTools/Point.h"
#include "GeometryTools/Speed.h"
#include "Navigation/Parameters.h"

namespace rd {
class PositionControlBase {
 public:
  PositionControlBase(const PositionControlParameters params) : params_(params) {}
  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) = 0;
  virtual bool isGoalReached() const = 0;

 protected:
  const PositionControlParameters params_;
};
}  // namespace rd

#endif /* POSITIONCONTROLBASE_H */
