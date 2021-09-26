#include <iostream>

#include "Navigation/Geometry/Point.h"
#include "Navigation/Geometry/Trajectory.h"
#include "Navigation/PositionControlBase.h"
#include "Navigation/RotationControl.h"

void say_hello() { std::cout << "Hello, from navigation!\n"; }

int main() {
  say_hello();
  rd::PointOriented robotPose(1.0, 0.0, 1.5);
  rd::Speed robotSpeed(2.0, 0.0, 0.0);
  rd::RotationControl rc;
  rc.setParam(rd::MAX_ROTATIONAL_ACCELERATION, 3.0);
  rc.setTargetAngle(2.0);
  for (size_t i = 0; i < 100; i++) {
    rd::Speed speed = rc.computeSpeed(robotPose, robotSpeed, 0.1);
    robotPose += rd::PointOriented(0.0, 0.0, speed.vtheta() * 0.1);
    std::cout << speed << std::endl;
    std::cout << robotPose << std::endl;
  }
}
