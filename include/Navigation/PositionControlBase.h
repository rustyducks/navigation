#ifndef POSITIONCONTROLBASE_H
#define POSITIONCONTROLBASE_H

#include <string>
#include <unordered_map>

#include "Navigation/Geometry/Point.h"
#include "Navigation/Geometry/Speed.h"

namespace rd {

enum ParamName {
  MAX_LINEAR_ACCELERATION = 0,
  MAX_LINEAR_SPEED,
  ADMITTED_LINEAR_POSITION_ERROR,
  MAX_ROTATIONAL_ACCELERATION,
  MAX_ROTATIONAL_SPEED,
  ADMITTED_ANGLE_POSITION_ERROR,
  PURE_PURSUIT_LOOKAHEAD_DISTANCE,
  LINEAR_CONTROL_STOP_DISTANCE_FACTOR,
  IS_HOLONOMIC
};

class UnknownParamError : public std::runtime_error {
 public:
  inline UnknownParamError(const ParamName paramName, const std::string& paramType)
      : std::runtime_error("Unknown param name '" + std::to_string(paramName) + "' for the wanted type: " + paramType + "."){};
};

class PositionControlBase {
 public:
  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) = 0;
  template <typename T>
  inline T param(const ParamName) const {
    throw std::runtime_error("Param type not supported");
  }
  template <typename T>
  inline void setParam(const ParamName, const T&) {
    throw std::runtime_error("Param type not supported");
  }

 private:
  std::unordered_map<ParamName, double> doubleParams_;
  std::unordered_map<ParamName, int> intParams_;
  std::unordered_map<ParamName, bool> boolParams_;
};
template <>
inline double PositionControlBase::param(const ParamName name) const {
  if (doubleParams_.find(name) == doubleParams_.end()) {
    throw UnknownParamError(name, "double");
  }
  return doubleParams_.at(name);
}
template <>
inline int PositionControlBase::param(const ParamName name) const {
  if (intParams_.find(name) == intParams_.end()) {
    throw UnknownParamError(name, "int");
  }
  return intParams_.at(name);
}
template <>
inline bool PositionControlBase::param(const ParamName name) const {
  if (boolParams_.find(name) == boolParams_.end()) {
    throw UnknownParamError(name, "bool");
  }
  return boolParams_.at(name);
}
template <>
inline void PositionControlBase::setParam(const ParamName name, const double& value) {
  doubleParams_[name] = value;
}
template <>
inline void PositionControlBase::setParam(const ParamName name, const int& value) {
  intParams_[name] = value;
}
template <>
inline void PositionControlBase::setParam(const ParamName name, const bool& value) {
  boolParams_[name] = value;
}
}  // namespace rd

#endif /* POSITIONCONTROLBASE_H */
