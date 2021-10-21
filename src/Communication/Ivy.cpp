#include "Navigation/Communication/Ivy.h"

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>

#include <sstream>
namespace rd {
Ivy::Ivy() {
  IvyInit("Navigation", "Navigation Online", nullptr, nullptr, nullptr, nullptr);
  IvyStart("127.255.255.255:2010");
  // ivyThread_ = std::thread(IvyMainLoop);
}

Ivy::~Ivy() {
  IvyStop();
  ivyThread_.join();
}

void Ivy::sendTrajectory(const Trajectory& trajectory) const {
  std::ostringstream oss;
  oss << "New trajectory ";
  for (size_t i = 0; i < trajectory.size(); i++) {
    oss << trajectory.at(i).point.x() << "," << trajectory.at(i).point.y();
    if (i < trajectory.size() - 1) {
      oss << ";";
    }
  }
  IvySendMsg("%s", oss.str().c_str());
}

void Ivy::sendPoint(const size_t id, const Point& point) const {
  std::ostringstream oss;
  oss << "Highlight point " << id << ";" << point.x() << ";" << point.y();
  IvySendMsg("%s", oss.str().c_str());
}

void Ivy::sendRobotPose(const PointOriented& robotPose) const {
  std::ostringstream oss;
  oss << "Update robot pose " << robotPose.x() << ";" << robotPose.y() << ";" << robotPose.theta().value();
  IvySendMsg("%s", oss.str().c_str());
}
}  // namespace rd