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

void Ivy::subscribeGoToOrient(const std::function<void(const PointOriented&)>& onGoToOrient) {
  if (onGoToOrientCbs_.size() == 0) {
    IvyBindMsg([](const IvyClientPtr, void*, int argc, char** argv) -> void { Ivy::getInstance().onGoToOrientBase(argc, argv); }, nullptr,
               "Go to orient (.*),(.*),(.*)");
  }
  onGoToOrientCbs_.push_back(onGoToOrient);
}

void Ivy::onGoToOrientBase(int argc, char** argv) const {
  if (argc != 3) {
    std::cout << "[Ivy] Received an ill formed 'GoToOrient' request. Discarding it." << std::endl;
  }
  double x, y, theta;
  sscanf(argv[0], "%lf", &x);
  sscanf(argv[1], "%lf", &y);
  sscanf(argv[2], "%lf", &theta);
  PointOriented p(x, y, theta);
  for (const auto cb : onGoToOrientCbs_) {
    cb(p);
  }
}

void Ivy::subscribeGoTo(const std::function<void(const Point&)>& onGoTo) {
  if (onGoToCbs_.size() == 0) {
    IvyBindMsg([](const IvyClientPtr, void*, int argc, char** argv) -> void { Ivy::getInstance().onGoToBase(argc, argv); }, nullptr, "Go to linear (.*),(.*)");
  }
  onGoToCbs_.push_back(onGoTo);
}

void Ivy::onGoToBase(int argc, char** argv) const {
  if (argc != 2) {
    std::cout << "[Ivy] Received an ill formed 'GoTo' request. Discarding it." << std::endl;
  }
  double x, y;
  sscanf(argv[0], "%lf", &x);
  sscanf(argv[1], "%lf", &y);
  Point p(x, y);
  for (const auto cb : onGoToCbs_) {
    cb(p);
  }
}

void Ivy::subscribeCustomAction(const std::function<void(const int)>& onCustomAction) {
  if (onCustomActionCbs_.size() == 0) {
    IvyBindMsg([](const IvyClientPtr, void*, int argc, char** argv) -> void { Ivy::getInstance().onCustomActionBase(argc, argv); }, nullptr,
               "Custom action (.*)");
  }
  onCustomActionCbs_.push_back(onCustomAction);
}

void Ivy::onCustomActionBase(int argc, char** argv) const {
  if (argc != 1) {
    std::cout << "[Ivy] Received an ill formed 'CustomAction' request. Discarding it." << std::endl;
  }
  int ca;
  sscanf(argv[0], "%d", &ca);
  for (const auto cb : onCustomActionCbs_) {
    cb(ca);
  }
}

void Ivy::subscribeSpeedDirection(const std::function<void(const Speed&)>& onSpeedDirection) {
  if (onSpeedDirectionCbs_.size() == 0) {
    IvyBindMsg([](const IvyClientPtr, void*, int argc, char** argv) -> void { Ivy::getInstance().onSpeedDirectionBase(argc, argv); }, nullptr,
               "Direction (.*),(.*),(.*)");
  }
  onSpeedDirectionCbs_.push_back(onSpeedDirection);
}

void Ivy::onSpeedDirectionBase(int argc, char** argv) const {
  if (argc != 3) {
    std::cout << "[Ivy] Received an ill formed 'SpeedDirection' request. Discarding it." << std::endl;
  }
  double vx, vy, vtheta;
  sscanf(argv[0], "%lf", &vx);
  sscanf(argv[1], "%lf", &vy);
  sscanf(argv[2], "%lf", &vtheta);
  Speed s(vx, vy, vtheta);
  for (const auto cb : onSpeedDirectionCbs_) {
    cb(s);
  }
}

}  // namespace rd
