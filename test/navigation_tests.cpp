#include <gtest/gtest.h>

#include <iostream>
#include <random>

#include "Navigation/Communication/Ivy.h"
#include "Navigation/Parameters.h"
#include "Navigation/PositionControlBase.h"
#include "Navigation/PurePursuitControl.h"
#include "Navigation/RotationControl.h"

using namespace rd;

void simulate(PointOriented& robotPose, const Speed& speed, double dt) {
  std::default_random_engine generator;
  double vxNoise = speed.vx() < 1. ? 0. : std::normal_distribution<double>(0.0, 7.5)(generator);
  double vthetaNoise = std::normal_distribution<double>(0.0, 0.003)(generator);
  robotPose +=
      PointOriented((speed.vx() + vxNoise) * robotPose.theta().cos() * dt - speed.vy() * robotPose.theta().sin() * dt,
                    (speed.vx() + vxNoise) * robotPose.theta().sin() * dt + speed.vy() * robotPose.theta().cos() * dt, (speed.vtheta() + vthetaNoise) * dt);
}

PositionControlParameters params = {
    50., 200., 0.5, 1.8, 5., 0.05,
};

class RotationControlTest : public ::testing::Test {
 protected:
  RotationControlTest() : ::testing::Test(), rc_(params) {}
  void SetUp() override{};
  RotationControl rc_;
};

TEST_F(RotationControlTest, Params) {
  PointOriented robotPose(1.0, 2.4, 0.0);
  Speed robotSpeed(0.0, 0.0, 0.0);
  double dt = 0.1;
  rc_.setTargetAngle(0.5);
  ASSERT_NO_THROW(rc_.computeSpeed(robotPose, robotSpeed, dt));
}

TEST_F(RotationControlTest, Control) {
  PointOriented robotPose(1.0, 2.4, 0.0);
  Speed robotSpeed(0.0, 0.0, 0.0);
  double dt = 0.1;
  rc_.setTargetAngle(2.5);
  Speed s;
  for (size_t i = 0; i < 5; i++) {
    robotSpeed = rc_.computeSpeed(robotPose, robotSpeed, dt);
    simulate(robotPose, robotSpeed, dt);
  }
  ASSERT_FLOAT_EQ(robotSpeed.vx(), 0.0);
  ASSERT_FLOAT_EQ(robotSpeed.vy(), 0.0);
  ASSERT_FLOAT_EQ(robotSpeed.vtheta(), 5 * MAX_ROTATIONAL_ACCELERATION * dt);
  for (size_t i = 0; i < 350; i++) {
    robotSpeed = rc_.computeSpeed(robotPose, robotSpeed, dt);
    simulate(robotPose, robotSpeed, dt);
    ASSERT_FLOAT_EQ(robotSpeed.vx(), 0.0);
    ASSERT_FLOAT_EQ(robotSpeed.vy(), 0.0);
    ASSERT_LE(robotSpeed.vtheta(), MAX_ROTATIONAL_SPEED);
  }
  ASSERT_FLOAT_EQ(robotSpeed.vx(), 0.0);
  ASSERT_FLOAT_EQ(robotSpeed.vy(), 0.0);
  ASSERT_FLOAT_EQ(robotSpeed.vtheta(), 0.0);
  ASSERT_FLOAT_EQ(robotPose.x(), 1.0);
  ASSERT_FLOAT_EQ(robotPose.y(), 2.4);
  ASSERT_LE(std::abs(robotPose.theta().value() - 2.5), ADMITTED_ANGLE_POSITION_ERROR);
  rc_.setTargetAngle(1.0);
  for (size_t i = 0; i < 10; i++) {
    robotSpeed = rc_.computeSpeed(robotPose, robotSpeed, dt);
    simulate(robotPose, robotSpeed, dt);
    ASSERT_LE(robotSpeed.vtheta(), 0.0);
  }
}

class PurePursuitControlTest : public ::testing::Test {
 protected:
  PurePursuitControlTest() : ::testing::Test(), pp_(params, 3, 50.) {}
  void SetUp() override{};
  PurePursuitControl pp_;
};

TEST_F(PurePursuitControlTest, Control) {
  Ivy& ivy = Ivy::getInstance();
  PointOriented robotPose(600.0, 730.0, 0.5);
  Speed robotSpeed(0.0, 0.0, 0.0);
  double dt = 0.1;
  Trajectory traj = Path({{600.0, 730.0, 0.5}, {1500.0, 900.0, 0.0}, {1600.0, 1000.0, 0.0}}).computeSpeeds();
  Trajectory traj2 = Path::lissajouPath(robotPose, 200).computeSpeeds();
  ivy.sendPath(traj2);
  pp_.setTrajectory(traj2);
  for (size_t i = 0; i < traj2.size(); i++) {
    std::cout << traj2.at(i).speed() << std::endl;
  }
  for (size_t i = 0; i < 2000; i++) {
    robotSpeed = pp_.computeSpeed(robotPose, robotSpeed, dt);
    simulate(robotPose, robotSpeed, dt);
    ivy.sendRobotPose(robotPose);
    // std::cout << robotPose << "  " << robotSpeed << std::endl;
    //  usleep(dt * 1000000);
    ASSERT_LE(robotSpeed.vx(), MAX_LINEAR_SPEED);
    if (pp_.isGoalReached()) {
      break;
    }
  }
}
/*TEST_F(PurePursuitControlTest, TrajTests) {
  Ivy& ivy = Ivy::getInstance();
  Trajectory traj({{600, 600, 0.}, {700, 700, 0}, {800, 700, 0}});
  Point other(500, 400);
  // double t;
  // size_t i;
  Point inter = traj.pointAtDistanceFrom(100, other);
  // Point inter = other.closestPointBetween(traj.at(0).point, traj.at(1).point, t);
  std::cout << inter << std::endl;
  for (size_t j = 0; j < 200; j++) {
    other += Point(10, 10);
    inter = traj.pointAtDistanceFrom(100, other);
    ivy.sendTrajectory(traj);
    ivy.sendPoint(0, other);
    ivy.sendPoint(1, inter);
    std::cout << inter << std::endl;
    usleep(500000);
  }
}*/
