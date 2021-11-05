#ifndef RD_IVY_H
#define RD_IVY_H
#include <thread>

#include "GeometryTools/Point.h"
#include "GeometryTools/Speed.h"
#include "GeometryTools/Trajectory.h"

namespace rd {
class Ivy {
 public:
  static Ivy& getInstance() {
    static Ivy instance;
    return instance;
  }

  void sendPath(const Path& path) const;
  void sendPoint(const size_t id, const Point& point) const;
  void sendRobotPose(const PointOriented& robotPose) const;

  void subscribeGoToOrient(const std::function<void(const PointOriented&)>& onGoToOrient);
  void subscribeGoTo(const std::function<void(const Point&)>& onGoTo);
  void subscribeCustomAction(const std::function<void(const int)>& onCustomAction);
  void subscribeSpeedDirection(const std::function<void(const Speed&)>& onSpeedDirection);

  void onGoToOrientBase(int argc, char** argv) const;
  void onGoToBase(int argc, char** argv) const;
  void onCustomActionBase(int argc, char** argv) const;
  void onSpeedDirectionBase(int argc, char** argv) const;

 protected:
  ~Ivy();
  Ivy();
  Ivy(const Ivy&);
  void operator=(const Ivy&);
  std::vector<std::function<void(const PointOriented&)>> onGoToOrientCbs_;
  std::vector<std::function<void(const Point&)>> onGoToCbs_;
  std::vector<std::function<void(const int)>> onCustomActionCbs_;
  std::vector<std::function<void(const Speed&)>> onSpeedDirectionCbs_;
  std::thread ivyThread_;
};
}  // namespace rd

#endif /* RD_IVY_H */
