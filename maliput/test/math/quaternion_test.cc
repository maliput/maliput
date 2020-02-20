#include "maliput/math/quaternion.h"

#include <gtest/gtest.h>

#include "maliput/math/matrix.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"
#include "maliput/test_utilities/maliput_math_compare.h"

namespace maliput {
namespace math {
namespace test {
namespace {

constexpr double kTolerance{1e-15};

GTEST_TEST(Quaternion, DefaultConstructor) {
  const Quaternion dut;
  EXPECT_EQ(dut.w(), 1.);
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.z(), 0.);
}

GTEST_TEST(Quaternion, Identity) {
  const Quaternion dut = Quaternion::Identity();
  EXPECT_EQ(dut.w(), 1.);
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.z(), 0.);
}

GTEST_TEST(Quaternion, CoefficientsConstructor) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);
}

GTEST_TEST(Quaternion, VectorConstructor) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  // Default constructor.
  const Quaternion dut(Vector4(kW, kX, kY, kZ));
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);
}

GTEST_TEST(Quaternion, RotationMatrixRoundTrip) {
  const double kMinAngle{-M_PI / 2.};
  const double kMaxAngle{kMinAngle + M_PI};
  const int kNAngles{5};
  const double kAngleStep{(kMaxAngle - kMinAngle) / static_cast<double>(kNAngles)};

  for (double roll = kMinAngle; roll <= kMaxAngle; roll += kAngleStep) {
    for (double pitch = kMinAngle; pitch <= kMaxAngle; pitch += kAngleStep) {
      for (double yaw = kMinAngle; yaw <= kMaxAngle; yaw += kAngleStep) {
        const Matrix3 kRotationMatrix = RollPitchYaw(roll, pitch, yaw).ToMatrix();
        const Quaternion dut(kRotationMatrix);
        EXPECT_TRUE(CompareMatrices(kRotationMatrix, dut.ToRotationMatrix(), kTolerance));
      }
    }
  }
}

GTEST_TEST(Quaternion, GettersAndSetters) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);

  Quaternion mutable_dut(kW, kX, kY, kZ);
  EXPECT_EQ(mutable_dut.w(), kW);
  EXPECT_EQ(mutable_dut.x(), kX);
  EXPECT_EQ(mutable_dut.y(), kY);
  EXPECT_EQ(mutable_dut.z(), kZ);

  mutable_dut.w() = mutable_dut.w() * 2.;
  mutable_dut.x() = mutable_dut.x() * 2.;
  mutable_dut.y() = mutable_dut.y() * 2.;
  mutable_dut.z() = mutable_dut.z() * 2.;
  EXPECT_EQ(mutable_dut.w(), 2. * kW);
  EXPECT_EQ(mutable_dut.x(), 2. * kX);
  EXPECT_EQ(mutable_dut.y(), 2. * kY);
  EXPECT_EQ(mutable_dut.z(), 2. * kZ);
}

GTEST_TEST(Quaternion, VectorAndCoefficients) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);

  EXPECT_TRUE(CompareVectors(Vector3(kX, kY, kZ), dut.vec(), 0. /* tolerance */));
  EXPECT_TRUE(CompareVectors(Vector4(kW, kX, kY, kZ), dut.coeffs(), 0. /* tolerance */));
}

GTEST_TEST(Quaternion, SetIdentity) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  Quaternion dut(kW, kX, kY, kZ);
  EXPECT_EQ(dut.w(), kW);
  EXPECT_EQ(dut.x(), kX);
  EXPECT_EQ(dut.y(), kY);
  EXPECT_EQ(dut.z(), kZ);

  dut.set_identity();
  EXPECT_EQ(dut.w(), 1.);
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.z(), 0.);
}

// Constants below were obtained with the following Python script:
//
// ```
// import math as m
//
// w = 0.884
// x = 0.306
// y = 0.177
// z = 0.306
//
// squared_norm = w*w + x*x + y*y + z*z
// norm = m.sqrt(squared_norm)
// scaled_squared_norm = 3*3*w*w + 3*3*x*x + 3*3*y*y + 3*3*z*z
// scaled_norm = m.sqrt(scaled_squared_norm)
// ```
GTEST_TEST(Quaternion, Norm) {
  EXPECT_NEAR(Quaternion::Identity().norm(), 1., kTolerance);

  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_NEAR(1.0000284995938866, dut.norm(), kTolerance);

  const Quaternion scaled_dut(3. * kW, 3. * kX, 3. * kY, 3. * kZ);
  EXPECT_NEAR(3.00008549878166, scaled_dut.norm(), kTolerance);
}

GTEST_TEST(Quaternion, SquaredNorm) {
  EXPECT_NEAR(Quaternion::Identity().squared_norm(), 1., kTolerance);

  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_NEAR(1.000057, dut.squared_norm(), kTolerance);

  const Quaternion scaled_dut(3. * kW, 3. * kX, 3. * kY, 3. * kZ);
  EXPECT_NEAR(9.000513000000002, scaled_dut.squared_norm(), kTolerance);
}

// Constants below were obtained with the following Python script:
//
// ```
// import math as m
//
// w = 1.
// x = 2.
// y = 3.
// z = 4.
//
// norm = m.sqrt(w*w + x*x + y*y + z*z)
// normalized_w = w / norm
// normalized_x = x / norm
// normalized_y = y / norm
// normalized_z = z / norm
// ```
GTEST_TEST(Quaternion, NormalizeAndNormalized) {
  const double kW{1.};
  const double kX{2.};
  const double kY{3.};
  const double kZ{4.};

  {
    Quaternion dut(kW, kX, kY, kZ);
    dut.normalize();
    EXPECT_NEAR(1., dut.norm(), kTolerance);
    EXPECT_TRUE(CompareVectors(Vector4(0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214),
                               dut.coeffs(), kTolerance));
  }
  {
    const Quaternion dut(kW, kX, kY, kZ);
    const Quaternion normalized_quaternion = dut.normalized();
    EXPECT_NEAR(1., normalized_quaternion.norm(), kTolerance);
    EXPECT_TRUE(CompareVectors(Vector4(0.18257418583505536, 0.3651483716701107, 0.5477225575051661, 0.7302967433402214),
                               normalized_quaternion.coeffs(), kTolerance));
  }
}

GTEST_TEST(Quaternion, DotProduct) {
  const double kA{1.};
  const double kB{2.};
  const double kC{3.};
  const double kD{4.};

  const Quaternion dut(kA, kB, kC, kD);
  EXPECT_NEAR(30., dut.dot(dut), kTolerance);
  EXPECT_NEAR(30., dut.squared_norm(), kTolerance);
  EXPECT_NEAR(20., dut.dot(Quaternion(kD, kC, kB, kA)), kTolerance);
}

GTEST_TEST(Quaternion, AngularDistance) {
  // On the same plane.
  {
    const Quaternion q1(M_PI / 2., Vector3(1., 0., 0.));
    const Quaternion q2(M_PI / 4., Vector3(1., 0., 0.));
    EXPECT_NEAR(q1.AngularDistance(q2), M_PI / 4., kTolerance);
  }
  // The conjugate rotation must be twice the path of the original.
  {
    const Quaternion q1(M_PI / 3., Vector3(1., 0., 0.));
    EXPECT_NEAR(q1.AngularDistance(q1.conjugate()), 2 * M_PI / 3., kTolerance);
  }
  // Distance to the same quaternion must be zero.
  {
    const Quaternion q1(M_PI / 3., Vector3(1., 0., 0.));
    EXPECT_NEAR(q1.AngularDistance(q1), 0., kTolerance);
  }
}

GTEST_TEST(Quaternion, EqualTo) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const double kAnyOtherValue{0.};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_TRUE(dut == dut);
  EXPECT_TRUE(dut == Quaternion(kW, kX, kY, kZ));
  EXPECT_FALSE(dut == Quaternion(kAnyOtherValue, kX, kY, kZ));
  EXPECT_FALSE(dut == Quaternion(kW, kAnyOtherValue, kY, kZ));
  EXPECT_FALSE(dut == Quaternion(kW, kX, kAnyOtherValue, kZ));
  EXPECT_FALSE(dut == Quaternion(kW, kX, kY, kAnyOtherValue));
}

GTEST_TEST(Quaternion, NotEqualTo) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const double kAnyOtherValue{0.};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_FALSE(dut != dut);
  EXPECT_FALSE(dut != Quaternion(kW, kX, kY, kZ));
  EXPECT_TRUE(dut != Quaternion(kAnyOtherValue, kX, kY, kZ));
  EXPECT_TRUE(dut != Quaternion(kW, kAnyOtherValue, kY, kZ));
  EXPECT_TRUE(dut != Quaternion(kW, kX, kAnyOtherValue, kZ));
  EXPECT_TRUE(dut != Quaternion(kW, kX, kY, kAnyOtherValue));
}

// Let $q$ be a quaternion, then its inverse is $ conj(q) / |q|^2 $. The
// following constants were derived in a python script using the previous
// definition.
//
// ```
// w = 0.884
// x = 0.306
// y = 0.177
// z = 0.306
//
// squared_norm = w*w + x*x + y*y + z*z
// w / squared_norm
// -x / squared_norm
// -y / squared_norm
// -z / squared_norm
// ```
GTEST_TEST(Quaternion, Inverse) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const double kAnyOtherValue{0.};

  const Quaternion dut(kW, kX, kY, kZ);
  EXPECT_TRUE(
      CompareVectors(Vector4(0.8839496148719523, -0.30598255899413734, -0.1769899115750402, -0.30598255899413734),
                     dut.Inverse().coeffs(), kTolerance));
  // Zero quaternion is handled separately up to Quaternion's tolerance.
  EXPECT_TRUE(CompareVectors(Vector4(0., 0., 0., 0.),
                             Quaternion(Quaternion::kTolerance / 2., 0., 0., 0.).Inverse().coeffs(), kTolerance));
  EXPECT_TRUE(CompareVectors(Vector4(0., 0., 0., 0.), Quaternion(Quaternion::kTolerance, 0., 0., 0.).Inverse().coeffs(),
                             kTolerance));
}

GTEST_TEST(Quaternion, Conjugate) {
  const double kW{0.884};
  const double kX{0.306};
  const double kY{0.177};
  const double kZ{0.306};
  const Quaternion dut = Quaternion(kW, kX, kY, kZ).conjugate();
  EXPECT_EQ(kW, dut.w());
  EXPECT_EQ(-kX, dut.x());
  EXPECT_EQ(-kY, dut.y());
  EXPECT_EQ(-kZ, dut.z());
}

GTEST_TEST(Quaternion, IsApprox) {
  const double kAlmostEqualPrecision{1e-3};
  const Vector3 kUnitX(1., 0., 0);
  const Vector3 kUnitY(0., 1., 0);

  {
    const Quaternion q1(0., Vector3(1., 0., 0.));
    const Quaternion q2(0., Vector3(0., 1., 0.));
    EXPECT_TRUE(q1.IsApprox(q2, kAlmostEqualPrecision));
  }
  {
    const Quaternion q1(0., kUnitX);
    // Adds a very tiny rotation around the axis.
    const Quaternion q2(1e-4, kUnitX);
    EXPECT_TRUE(q1.IsApprox(q2, kAlmostEqualPrecision));
  }
  {
    const Quaternion q1(0., kUnitX);
    const Quaternion q2(M_PI / 3., (kUnitX + kUnitY).normalized());
    EXPECT_FALSE(q1.IsApprox(q2, kAlmostEqualPrecision));
  }
}

GTEST_TEST(Quaternion, MultiplicationBetweenQuaternions) {
  const Vector3 kUnitX(1., 0., 0.);
  const Vector3 kUnitY(0., 1., 0.);
  const Vector3 kUnitZ(0., 0., 1.);
  const Vector3 kAnyDirection(1., 2., 3.);

  const Quaternion q1 = Quaternion::FromTwoVectors(kUnitX, kUnitY);
  const Quaternion q2 = Quaternion::FromTwoVectors(kUnitY, kUnitX);
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), (q1 * q2).coeffs(), kTolerance));

  const Quaternion q3 = Quaternion::FromTwoVectors(kUnitZ, kAnyDirection);
  const Quaternion q4 = Quaternion::FromTwoVectors(kAnyDirection, kUnitZ);
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), (q3 * q4).coeffs(), kTolerance));
}

GTEST_TEST(Quaternion, MultiplicationAndAssingmentBetweenQuaternions) {
  const Vector3 kUnitX(1., 0., 0.);
  const Vector3 kUnitY(0., 1., 0.);
  const Vector3 kUnitZ(0., 0., 1.);
  const Vector3 kAnyDirection(1., 2., 3.);

  const Quaternion q1 = Quaternion::FromTwoVectors(kUnitX, kUnitY);
  Quaternion q2 = Quaternion::FromTwoVectors(kUnitY, kUnitX);
  q2 *= q1;
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), q2.coeffs(), kTolerance));

  const Quaternion q3 = Quaternion::FromTwoVectors(kUnitZ, kAnyDirection);
  Quaternion q4 = Quaternion::FromTwoVectors(kAnyDirection, kUnitZ);
  q4 *= q3;
  EXPECT_TRUE(CompareVectors(Quaternion::Identity().coeffs(), q4.coeffs(), kTolerance));
}

GTEST_TEST(Quaternion, TransformVector) {
  const Vector3 v(-1., 1., 1.);
  const Quaternion dut(1., 0., 1., 0.);
  const Vector3 kExpectedResult(3., 1., 1.);

  EXPECT_TRUE(CompareVectors(kExpectedResult, dut.TransformVector(v), kTolerance));
}

GTEST_TEST(Quaternion, MultiplicationByVector) {
  const Vector3 v(-1., 1., 1.);
  const Quaternion dut(1., 0., 1., 0.);
  const Vector3 kExpectedResult(3., 1., 1.);

  EXPECT_TRUE(CompareVectors(kExpectedResult, dut * v, kTolerance));
}

GTEST_TEST(Quaternion, Slerp) {
  const Quaternion q1(RollPitchYaw(0.1, 1.2, 2.3).ToMatrix());
  const Quaternion q2(RollPitchYaw(1.2, 2.3, -3.4).ToMatrix());

  const Quaternion q3 = q1.Slerp(1., q2);

  // Degrading tolerance in this test on purpose because of the overhead in the
  // computation and the expected result.
  EXPECT_TRUE(CompareVectors(Vector4(0.554528, -0.717339, 0.32579, 0.267925), q3.coeffs(), 1e-6 /* tolerance */));
}

GTEST_TEST(Quaternion, Serialization) {
  const Quaternion dut(1., 2., 3., 4.);
  std::stringstream ss;
  ss << dut;
  EXPECT_EQ(ss.str(), "(w: 1, x: 2, y: 3, z: 4)");
}

GTEST_TEST(Quaternion, FromTwoVectors) {
  const Vector3 kUnitX(1., 0., 0.);
  const Vector3 kUnitY(0., 1., 0.);
  const Vector3 kUnitZ(0., 0., 1.);

  const double kW = std::sqrt(2.) / 2.;
  const double kU = std::sqrt(2.) / 2.;

  {  // X to Y.
    const Quaternion dut = Quaternion::FromTwoVectors(kUnitX, kUnitY);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., 0., kU), dut.coeffs(), kTolerance));
  }
  {
    Quaternion dut;
    dut.SetFromTwoVectors(kUnitX, kUnitY);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., 0., kU), dut.coeffs(), kTolerance));
  }
  {  // Y to Z.
    const Quaternion dut = Quaternion::FromTwoVectors(kUnitY, kUnitZ);
    EXPECT_TRUE(CompareVectors(Vector4(kW, kU, 0., 0.), dut.coeffs(), kTolerance));
  }
  {
    Quaternion dut;
    dut.SetFromTwoVectors(kUnitY, kUnitZ);
    EXPECT_TRUE(CompareVectors(Vector4(kW, kU, 0., 0.), dut.coeffs(), kTolerance));
  }
  {  // Z to X.
    const Quaternion dut = Quaternion::FromTwoVectors(kUnitZ, kUnitX);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., kU, 0.), dut.coeffs(), kTolerance));
  }
  {
    Quaternion dut;
    dut.SetFromTwoVectors(kUnitZ, kUnitX);
    EXPECT_TRUE(CompareVectors(Vector4(kW, 0., kU, 0.), dut.coeffs(), kTolerance));
  }
}

}  // namespace
}  // namespace test
}  // namespace math
}  // namespace maliput
