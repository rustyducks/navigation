#include "Navigation/Communication/Ivy.h"

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>

#include <sstream>
namespace rd {

void quit(TimerId, void*, unsigned long) {}

Ivy::Ivy() {
  char name[] = "Navigation";
  IvyInit(name, "Navigation Online", nullptr, nullptr, nullptr, nullptr);
  IvyStart("127.255.255.255:2010");
  TimerRepeatAfter(TIMER_LOOP, 1000, quit, 0);  // Hack for Ivy, makes Ivy loop actually loop and check for ivy stop.
  ivyThread_ = std::thread(IvyMainLoop);
}

Ivy::~Ivy() {
  IvyStop();
  ivyThread_.join();
}

void Ivy::sendPath(const Path& path) const {
  std::ostringstream oss;
  oss << "New trajectory ";
  for (size_t i = 0; i < path.size(); i++) {
    oss << path.at(i).x() << "," << path.at(i).y();
    if (i < path.size() - 1) {
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