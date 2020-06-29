#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/test_utilities/maliput_math_compare.h"

#include <gtest/gtest.h>

namespace maliput {
namespace math {
namespace {

static constexpr double kTolerance{1e-10};

void ExpectDoubleEq(const RollPitchYaw& rpy1, const RollPitchYaw& rpy2) {
  EXPECT_DOUBLE_EQ(rpy1.roll_angle(), rpy2.roll_angle());
  EXPECT_DOUBLE_EQ(rpy1.pitch_angle(), rpy2.pitch_angle());
  EXPECT_DOUBLE_EQ(rpy1.yaw_angle(), rpy2.yaw_angle());
}

void ExpectDoubleEq(const Quaternion& q1, const Quaternion& q2) {
  EXPECT_DOUBLE_EQ(q1.w(), q2.w());
  EXPECT_DOUBLE_EQ(q1.x(), q2.x());
  EXPECT_DOUBLE_EQ(q1.y(), q2.y());
  EXPECT_DOUBLE_EQ(q1.z(), q2.z());
}

GTEST_TEST(RollPitchYawTest, Constructors) {
  ExpectDoubleEq(RollPitchYaw{}, RollPitchYaw(Vector3::Zero()));
  ExpectDoubleEq(RollPitchYaw(0.1, -0.2, 0.3), RollPitchYaw(Vector3{0.1, -0.2, 0.3}));
  {
    const RollPitchYaw kExpectedDut{0.4, -0.5, 0.6};
    const RollPitchYaw kDut = kExpectedDut;
    ExpectDoubleEq(kDut, kExpectedDut);
  }
  {
    const RollPitchYaw kDut = RollPitchYaw{0.4, -0.5, 0.6};
    ExpectDoubleEq(kDut, RollPitchYaw(0.4, -0.5, 0.6));
  }
}

GTEST_TEST(RollPitchYawTest, PublicMethods) {
  // accessors, ToMatrix, ToQuaternion with Identity rotation
  {
    const RollPitchYaw kDut;
    EXPECT_EQ(kDut.vector(), Vector3(0., 0., 0.));
    EXPECT_DOUBLE_EQ(kDut.roll_angle(), 0.);
    EXPECT_DOUBLE_EQ(kDut.pitch_angle(), 0.);
    EXPECT_DOUBLE_EQ(kDut.yaw_angle(), 0.);
    EXPECT_EQ(kDut.ToMatrix(), Matrix3::Identity());
    ExpectDoubleEq(kDut.ToQuaternion(), Quaternion::Identity());
  }
  // accessors, ToMatrix, ToQuaternion with non-Identity rotation
  {
    const RollPitchYaw kDut{M_PI / 2, 0., M_PI / 2};
    EXPECT_EQ(kDut.vector(), Vector3(M_PI / 2, 0., M_PI / 2));
    EXPECT_DOUBLE_EQ(kDut.roll_angle(), M_PI / 2);
    EXPECT_DOUBLE_EQ(kDut.pitch_angle(), 0.);
    EXPECT_DOUBLE_EQ(kDut.yaw_angle(), M_PI / 2);
    EXPECT_TRUE(
        test::CompareMatrices(kDut.ToMatrix(), Matrix3({{0., 0., 1.}, {1., 0., 0.}, {0., 1., 0.}}), kTolerance));
    ExpectDoubleEq(kDut.ToQuaternion(), Quaternion(0.5, 0.5, 0.5, 0.5));
  }
  // mutable references
  {
    const RollPitchYaw kExpectedDut{0.1, -0.2, 0.3};
    RollPitchYaw kDut;
    kDut.roll_angle() = 0.1;
    kDut.pitch_angle() = -0.2;
    kDut.yaw_angle() = 0.3;
    ExpectDoubleEq(kDut, kExpectedDut);
  }
  // set(double, double, double)
  {
    const RollPitchYaw kExpectedDut{0.1, -0.2, 0.3};
    RollPitchYaw kDut;
    kDut.set(0.1, -0.2, 0.3);
    ExpectDoubleEq(kDut, kExpectedDut);
  }
  // set(Vector3)
  {
    const Vector3 kExpectedDut{0.1, -0.2, 0.3};
    RollPitchYaw kDut;
    kDut.set(kExpectedDut);
    EXPECT_EQ(kDut.vector(), kExpectedDut);
  }
  // SetFromQuaternion
  {
    const RollPitchYaw kExpectedDut{M_PI / 2., 0., M_PI / 2.};
    const Quaternion q{0.5, 0.5, 0.5, 0.5};
    RollPitchYaw kDut;
    kDut.SetFromQuaternion(q);
    ExpectDoubleEq(kDut, kExpectedDut);
    ExpectDoubleEq(kDut.ToQuaternion(), q);
  }
}

// For a rotation matrix R that depends on roll-pitch-yaw angles `rpy`,
// calculate the ordinary derivative of R with respect to t.
GTEST_TEST(RollPitchYawTest, OrdinaryDerivativeRotationMatrixRollPitchYaw) {
  const RollPitchYaw rpy{0.2, 0.3, 0.4};
  const Vector3 rpyDt{-2.1, 3.3, 5.7};
  const Matrix3 RDt = rpy.CalcRotationMatrixDt(rpyDt);

  // Results generated by MotionGenesis.
  const double r = rpy.roll_angle();
  const double p = rpy.pitch_angle();
  const double y = rpy.yaw_angle();
  const double c0 = cos(r), c1 = cos(p), c2 = cos(y);
  const double s0 = sin(r), s1 = sin(p), s2 = sin(y);
  const double rDt = rpyDt.x(), pDt = rpyDt.y(), yDt = rpyDt.z();
  // clang-format off
  const Matrix3 MDt({-s1*c2*pDt - s2*c1*yDt,
                     s0*s2*rDt + s1*c0*c2*rDt + s0*c1*c2*pDt - c0*c2*yDt - s1*s0*s2*yDt,
                     s0*c2*yDt + s2*c0*rDt + c1*c0*c2*pDt - s1*s0*c2*rDt - s1*s2*c0*yDt,
                     c1*c2*yDt - s1*s2*pDt,
                     s1*s0*c2*yDt + s1*s2*c0*rDt + s0*s2*c1*pDt - s0*c2*rDt - s2*c0*yDt,
                     s0*s2*yDt + s1*c0*c2*yDt + s2*c1*c0*pDt - c0*c2*rDt - s1*s0*s2*rDt,
                     -c1*pDt,
                     c1*c0*rDt - s1*s0*pDt,
                     -s1*c0*pDt - s0*c1*rDt});
  // clang-format on

  EXPECT_TRUE(test::CompareMatrices(RDt, MDt, kTolerance));
}

}  // namespace
}  // namespace math
}  // namespace maliput
