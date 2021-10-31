#ifndef RD_IVY_H
#define RD_IVY_H
#include <thread>

#include "GeometryTools/Point.h"
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

 protected:
  ~Ivy();
  Ivy();
  Ivy(const Ivy&);
  void operator=(const Ivy&);
  std::thread ivyThread_;
};
}  // namespace rd

#endif /* RD_IVY_H */
