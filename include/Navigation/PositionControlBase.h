#ifndef POSITIONCONTROLBASE_H
#define POSITIONCONTROLBASE_H

#include <string>
#include <unordered_map>

#include "Navigation/Geometry/Point.h"
#include "Navigation/Geometry/Speed.h"

namespace rd {

enum ParamName { MAX_LINEAR_ACCELERATION = 0, MAX_LINEAR_SPEED, ADMITTED_LINEAR_POSITION_ERROR, MAX_ROTATIONAL_ACCELERATION, MAX_ROTATIONAL_SPEED, ADMITTED_ANGLE_POSITION_ERROR };

class UnknownParamError : public std::runtime_error {
 public:
  inline UnknownParamError(const ParamName paramName, const std::string& paramType) : std::runtime_error("Unknown param name '" + std::to_string(paramName) + "' for the wanted type: " + paramType + "."){};
};

class PositionControlBase {
 public:
  virtual Speed computeSpeed(const PointOriented& robotPose, const Speed& robotSpeed, double dt) = 0;
  template <typename T>
  inline T param(const ParamName name) const {
    throw std::runtime_error("Param type not supported");
  };
  template <typename T>
  inline void setParam(const ParamName name, const T& value) {
    throw std::runtime_error("Param type not supported");
  };

 private:
  std::unordered_map<ParamName, double> doubleParams_;
  std::unordered_map<ParamName, int> intParams_;
};
template <>
inline double PositionControlBase::param(const ParamName name) const {
  if (doubleParams_.find(name) == doubleParams_.end()) {
    throw UnknownParamError(name, "double");
  }
  return doubleParams_.at(name);
};
template <>
inline int PositionControlBase::param(const ParamName name) const {
  if (intParams_.find(name) == intParams_.end()) {
    throw UnknownParamError(name, "int");
  }
  return intParams_.at(name);
};
template <>
inline void PositionControlBase::setParam(const ParamName name, const double& value) {
  doubleParams_[name] = value;
};
template <>
inline void PositionControlBase::setParam(const ParamName name, const int& value) {
  intParams_[name] = value;
};
}  // namespace rd

#endif /* POSITIONCONTROLBASE_H */
