#include "Navigation/RotationControl.h"

namespace rd {

RotationControl::RotationControl(){};

Speed RotationControl::computeSpeed(const PointOriented &robotPose, const Speed &robotSpeed) { return Speed(0.0, 0.0, 0.0); }
} // namespace rd
