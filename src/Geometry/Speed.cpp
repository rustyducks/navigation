#include "Navigation/Geometry/Speed.h"

namespace rd {
Speed::Speed(double vx, double vy, double vtheta) : linearSpeed_(vx, vy), rotationalSpeed_(vtheta) {}
} // namespace rd
