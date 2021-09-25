#ifndef POSITIONCONTROLBASE_H
#define POSITIONCONTROLBASE_H

#include "Navigation/Geometry/Point.h"
#include "Navigation/Geometry/Speed.h"

namespace rd {
class PositionControlBase {
public:
  virtual Speed computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed) = 0;
};
} // namespace rd

#endif /* POSITIONCONTROLBASE_H */
