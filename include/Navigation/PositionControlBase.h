#ifndef POSITIONCONTROLBASE_H
#define POSITIONCONTROLBASE_H

#include <string>
#include <unordered_map>

#include "GeometryTools/Point.h"
#include "GeometryTools/Speed.h"

namespace rd {
class PositionControlBase {
 public:
  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) = 0;
};
}  // namespace rd

#endif /* POSITIONCONTROLBASE_H */
