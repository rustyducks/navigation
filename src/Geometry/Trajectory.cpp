#include "Navigation/Geometry/Trajectory.h"

namespace rd {
Trajectory::Trajectory() : pointspeeds_({}){};

Trajectory::Trajectory(const std::vector<PointOriented> &points) {}

const PointOrientedSpeed &Trajectory::at(size_t i) const { return pointspeeds_.at(i); }

} // namespace rd