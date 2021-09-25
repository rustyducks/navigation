#include <iostream>

#include "Navigation/Geometry/Point.h"
#include "Navigation/Geometry/Trajectory.h"

void say_hello() { std::cout << "Hello, from navigation!\n"; }

int main() {
  say_hello();
  rd::Trajectory t;
  rd::Point *p = new rd::PointOriented(10., 9., 2.4);
  std::cout << *static_cast<rd::PointOriented *>(p) << std::endl;
}
