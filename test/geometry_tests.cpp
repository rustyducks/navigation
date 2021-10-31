#include <gtest/gtest.h>

#include "GeometryTools/Point.h"
#include "GeometryTools/Trajectory.h"

using namespace rd;

class GeometryTest : public ::testing::Test {
 protected:
  GeometryTest() : ::testing::Test() {}
  void SetUp() override{};
};

TEST(GeometryTest, AngleTests) {
  Angle a(1.23);
  ASSERT_FLOAT_EQ(a.cos(), 0.3342377271245026);
  ASSERT_FLOAT_EQ(a.sin(), 0.9424888019316975);
  Angle a2 = a * 2;
  ASSERT_FLOAT_EQ(a2.value(), 2.46);
  ASSERT_TRUE(a2 > a);
  a2 *= 0.5;
  ASSERT_FLOAT_EQ(a2.value(), 1.23);
  ASSERT_TRUE(a2 == a);
  a2 += a * 3;
  ASSERT_FLOAT_EQ(a2.value(), -1.3631853071795863);
  ASSERT_TRUE(a2 < a);
}

TEST(GeometryTest, PointTests) {
  Point p(1.4, 1.0);
  ASSERT_FLOAT_EQ(p.x(), 1.4);
  ASSERT_FLOAT_EQ(p.y(), 1.0);
  ASSERT_FLOAT_EQ(p.norm(), std::hypot(1.4, 1.0));
  ASSERT_EQ(p.polarAngle(), Angle(0.6202494859828215));

  PointOriented frame(1.4, 1.0, 3.0), frame2(0.0, 0.0, -M_PI), frame3(0.0, 0.0, M_PI_2), frame4(2.3, 1.5, 1.98);
  ASSERT_EQ(p.transformIn(frame), Point(0.0, 0.0));
  Point frame22p = p.transformIn(frame2);
  ASSERT_FLOAT_EQ(frame22p.x(), -p.x());
  ASSERT_FLOAT_EQ(frame22p.y(), -p.y());
  Point frame32p = p.transformIn(frame3);
  ASSERT_FLOAT_EQ(frame32p.x(), p.y());
  ASSERT_FLOAT_EQ(frame32p.y(), -p.x());
  Point frame42p = p.transformIn(frame4);
  ASSERT_FLOAT_EQ(frame42p.x(), -0.10062799);
  ASSERT_FLOAT_EQ(frame42p.y(), 1.0246336);
}

TEST(GeometryTest, PointOrientedTests) {
  PointOriented po(3.6, -3.4, -20.43);
  ASSERT_FLOAT_EQ(po.x(), 3.6);
  ASSERT_FLOAT_EQ(po.y(), -3.4);
  ASSERT_FLOAT_EQ(po.theta().value(), -1.580444078461241);
  ASSERT_EQ(po.theta(), Angle(-7.863629385640827));
}

TEST(GeometryTest, TrajectoryTests) {
  Path t({{0.0, 0.0, 0.0}, {200.0, 200.0, 0.0}, {300.0, 200.0, 0.0}});
  size_t previousI;
  Point p = t.pointAtDistanceFrom(50., {100., 100.}, previousI);
  ASSERT_EQ(previousI, 0);
  ASSERT_FLOAT_EQ(p.x(), 135.35535);
  ASSERT_FLOAT_EQ(p.y(), 135.35535);
  p = t.pointAtBackwardDistanceFrom(50., {100.0, 100.0}, previousI);
  ASSERT_EQ(previousI, 0);
  ASSERT_FLOAT_EQ(p.x(), 64.644661);
  ASSERT_FLOAT_EQ(p.y(), 64.644661);
  p = t.pointAtDistanceFrom(40., {198.0, 198.0}, previousI);
  ASSERT_EQ(previousI, 1);
  p = t.pointAtBackwardDistanceFrom(50., {205.0, 200.0}, previousI);
  ASSERT_EQ(previousI, 0);
}