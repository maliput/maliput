// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput/api/lane_data.h"

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/math/matrix.h"
#include "maliput/math/quaternion.h"
#include "maliput/math/vector.h"
#include "maliput/test_utilities/maliput_math_compare.h"
#include "maliput/test_utilities/maliput_types_compare.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace api {
namespace {

static constexpr double kX0 = 23.;
static constexpr double kX1 = 75.;
static constexpr double kX2 = 0.567;

#define CHECK_ALL_LANE_POSITION_ACCESSORS(dut, _s, _r, _h) \
  do {                                                     \
    EXPECT_EQ(dut.s(), _s);                                \
    EXPECT_EQ(dut.r(), _r);                                \
    EXPECT_EQ(dut.h(), _h);                                \
    EXPECT_EQ(static_cast<int>(dut.srh().size()), 3);      \
    EXPECT_EQ(dut.srh().x(), _s);                          \
    EXPECT_EQ(dut.srh().y(), _r);                          \
    EXPECT_EQ(dut.srh().z(), _h);                          \
  } while (0)

GTEST_TEST(LanePositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  const LanePosition dut;
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 0., 0., 0.);
}

GTEST_TEST(LanePositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  const LanePosition dut(kX0, kX1, kX2);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
}

GTEST_TEST(LanePositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  const LanePosition dut = LanePosition::FromSrh(math::Vector3(kX0, kX1, kX2));
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
}

GTEST_TEST(LanePositionTest, VectorSetter) {
  // Check the vector-based setter.
  LanePosition dut(kX0, kX1, kX2);
  const math::Vector3 srh(9., 7., 8.);
  dut.set_srh(srh);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, srh.x(), srh.y(), srh.z());
}

GTEST_TEST(LanePositionTest, ComponentSetters) {
  // Check the individual component setters.
  LanePosition dut(0.1, 0.2, 0.3);

  dut.set_s(99.);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 99., 0.2, 0.3);

  dut.set_r(2.3);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 99., 2.3, 0.3);

  dut.set_h(42.);
  CHECK_ALL_LANE_POSITION_ACCESSORS(dut, 99., 2.3, 42.);
}

#undef CHECK_ALL_LANE_POSITION_ACCESSORS

// TODO(jadecastro) Use CompareMatrices() to implement the
// InertialPosition::xyz() accessor checks once AutoDiff supported.
#define CHECK_ALL_GEO_POSITION_ACCESSORS(dut, _x, _y, _z) \
  do {                                                    \
    EXPECT_EQ(dut.x(), _x);                               \
    EXPECT_EQ(dut.y(), _y);                               \
    EXPECT_EQ(dut.z(), _z);                               \
    EXPECT_EQ(static_cast<int>(dut.xyz().size()), 3);     \
    EXPECT_EQ(dut.xyz().x(), _x);                         \
    EXPECT_EQ(dut.xyz().y(), _y);                         \
    EXPECT_EQ(dut.xyz().z(), _z);                         \
  } while (0)

GTEST_TEST(InertialPositionTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  const InertialPosition dut;
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 0., 0., 0.);
}

GTEST_TEST(InertialPositionTest, ParameterizedConstructor) {
  // Check the fully-parameterized constructor.
  const InertialPosition dut(kX0, kX1, kX2);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
}

GTEST_TEST(InertialPositionTest, ConstructionFromVector) {
  // Check the conversion-construction from a 3-vector.
  const InertialPosition dut = InertialPosition::FromXyz(math::Vector3(kX0, kX1, kX2));
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, kX0, kX1, kX2);
}

GTEST_TEST(InertialPositionTest, VectorSetter) {
  // Check the vector-based setter.
  InertialPosition dut(kX0, kX1, kX2);
  const math::Vector3 xyz(9., 7., 8.);
  dut.set_xyz(xyz);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, xyz.x(), xyz.y(), xyz.z());
}

GTEST_TEST(InertialPositionTest, ComponentSetters) {
  // Check the individual component setters.
  InertialPosition dut(0.1, 0.2, 0.3);

  dut.set_x(99.);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 99., 0.2, 0.3);

  dut.set_y(2.3);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 99., 2.3, 0.3);

  dut.set_z(42.);
  CHECK_ALL_GEO_POSITION_ACCESSORS(dut, 99., 2.3, 42.);
}

#undef CHECK_ALL_GEO_POSITION_ACCESSORS

GTEST_TEST(InertialPositionTest, EqualityInequalityOperators) {
  // Checks that equality is true iff the constituent components are all equal,
  // and that inequality is true otherwise.
  const InertialPosition gp1(0.1, 0.2, 0.3);
  const InertialPosition gp2(0.1, 0.2, 0.3);

  EXPECT_TRUE(gp1 == gp2);
  EXPECT_FALSE(gp1 != gp2);

  const InertialPosition gp_xerror(gp2.x() + 1e-6, gp2.y(), gp2.z());
  EXPECT_FALSE(gp1 == gp_xerror);
  EXPECT_TRUE(gp1 != gp_xerror);
  const InertialPosition gp_yerror(gp2.x(), gp2.y() + 1e-6, gp2.z());
  EXPECT_FALSE(gp1 == gp_yerror);
  EXPECT_TRUE(gp1 != gp_xerror);
  const InertialPosition gp_zerror(gp2.x(), gp2.y(), gp2.z() + 1e-6);
  EXPECT_FALSE(gp1 == gp_zerror);
  EXPECT_TRUE(gp1 != gp_xerror);
}

GTEST_TEST(InertialPositionTest, Length) {
  // Check length of the vector.
  const InertialPosition dut(3., 4., 0.);
  EXPECT_EQ(dut.length(), 5.);
}

GTEST_TEST(InertialPositionTest, VectorArithmeticOperators) {
  // Check vector arithmetic operators +/- and multiply by scalar.
  const InertialPosition gp1(1., 2., 3.);
  const InertialPosition gp2(4., 5., 6.);

  InertialPosition dut = gp1 + gp2;
  EXPECT_TRUE(dut == InertialPosition(5., 7., 9.));
  dut = gp1 - gp2;
  EXPECT_TRUE(dut == InertialPosition(-3., -3., -3.));
  dut = 2. * gp1;
  EXPECT_TRUE(dut == InertialPosition(2., 4., 6.));
  dut = gp1 * 3.;
  EXPECT_TRUE(dut == InertialPosition(3., 6., 9.));
}

GTEST_TEST(InertialPosition, DistanceTest) {
  // Check the `Distance` method.
  // Test compared to the following python script.
  // >>> import math as m
  // >>> print(m.sqrt((66-25)**2 + ((90-85)**2) + ((-25-12)**2)))
  const double kLinearTolerance = 1e-15;
  const InertialPosition dut(25., 85., 12.);
  EXPECT_NEAR(dut.Distance({66.0, 90.0, -25.0}), 55.452682532047085, kLinearTolerance);
}

// Tests RoadPosition::ToInertialPosition method.
GTEST_TEST(RoadPosition, ToInertialPositionTest) {
  const LanePosition pos_1(0.0, kX1, kX2);
  const LanePosition pos_2(1.0, kX1, kX2);

  const InertialPosition start_ip(1, 0, 0);
  const InertialPosition end_ip(2, 0, 0);

  const maliput::api::test::MockLane lane{maliput::api::LaneId{"mock"}, start_ip, {}, end_ip, {}};

  RoadPosition dut(&lane, pos_1);

  EXPECT_EQ(start_ip, dut.ToInertialPosition());

  dut.pos = pos_2;

  EXPECT_EQ(end_ip, dut.ToInertialPosition());
}

// An arbitrary very small number (that passes the tests).
const double kRotationTolerance = 1e-15;

// TODO(francocipollone): Once RollPitchYaw and Quaternion implementation are
//                        complete the followings arguments of the CompareVectors could be modified.
#define CHECK_ALL_ROTATION_ACCESSORS(dut, _w, _x, _y, _z, _ro, _pi, _ya, _ma)                                        \
  do {                                                                                                               \
    EXPECT_TRUE(math::test::CompareVectors(dut.quat().coeffs(), math::Vector4(_w, _x, _y, _z), kRotationTolerance)); \
    EXPECT_TRUE(math::test::CompareVectors(                                                                          \
        math::Vector3(dut.rpy().roll_angle(), dut.rpy().pitch_angle(), dut.rpy().yaw_angle()),                       \
        math::Vector3(_ro, _pi, _ya), kRotationTolerance));                                                          \
    EXPECT_NEAR(dut.roll(), _ro, kRotationTolerance);                                                                \
    EXPECT_NEAR(dut.pitch(), _pi, kRotationTolerance);                                                               \
    EXPECT_NEAR(dut.yaw(), _ya, kRotationTolerance);                                                                 \
    EXPECT_TRUE(math::test::CompareMatrices(dut.matrix(), _ma, kRotationTolerance));                                 \
  } while (0)

class RotationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // A quaternion that rotates x->y, y->z, z->x...
    twist_quat_ = math::Quaternion(M_PI * 2. / 3., math::Vector3(1.0, 1.0, 1.0).normalized());
    nonnormalized_twist_quat_ = math::Quaternion(7. * twist_quat_.coeffs());

    twist_roll_ = M_PI / 2.;
    twist_pitch_ = 0.;
    twist_yaw_ = M_PI / 2.;

    twist_matrix_ = {0., 0., 1., 1., 0., 0., 0., 1., 0.};
  }

  math::Quaternion nonnormalized_twist_quat_;
  math::Quaternion twist_quat_;
  double twist_roll_;
  double twist_pitch_;
  double twist_yaw_;
  math::Matrix3 twist_matrix_;
};

TEST_F(RotationTest, DefaultConstructor) {
  // Check that default constructor obeys its contract.
  Rotation dut;
  CHECK_ALL_ROTATION_ACCESSORS(dut, 1., 0., 0., 0., 0., 0., 0., math::Matrix3::Identity());
}

TEST_F(RotationTest, ConstructionFromQuaternion) {
  // Check the conversion-construction from a Quaternion.
  Rotation dut = Rotation::FromQuat(nonnormalized_twist_quat_);
  CHECK_ALL_ROTATION_ACCESSORS(dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(), twist_roll_,
                               twist_pitch_, twist_yaw_, twist_matrix_);
}

TEST_F(RotationTest, ConstructionFromRpyVector) {
  // Check the conversion-construction from a 3-vector of roll, pitch, yaw.
  Rotation dut = Rotation::FromRpy({twist_roll_, twist_pitch_, twist_yaw_});
  CHECK_ALL_ROTATION_ACCESSORS(dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(), twist_roll_,
                               twist_pitch_, twist_yaw_, twist_matrix_);
}

TEST_F(RotationTest, ConstructionFromRpyComponents) {
  // Check the conversion-construction from individual roll, pitch, yaw.
  Rotation dut = Rotation::FromRpy(twist_roll_, twist_pitch_, twist_yaw_);
  CHECK_ALL_ROTATION_ACCESSORS(dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(), twist_roll_,
                               twist_pitch_, twist_yaw_, twist_matrix_);
}

TEST_F(RotationTest, QuaternionSetter) {
  // Check the vector-based setter.
  Rotation dut = Rotation::FromRpy(23., 75., 0.567);
  dut.set_quat(nonnormalized_twist_quat_);
  CHECK_ALL_ROTATION_ACCESSORS(dut, twist_quat_.w(), twist_quat_.x(), twist_quat_.y(), twist_quat_.z(), twist_roll_,
                               twist_pitch_, twist_yaw_, twist_matrix_);
}

GTEST_TEST(Rotation, ApplyTest) {
  // Results in this test were derived with the following Python script:
  // >>> import math as m
  // >>> import numpy as np
  // >>> def rotation_matrix (roll, pitch ,yaw):
  // >>>     ca_r = m.cos(roll)
  // >>>     sa_r = m.sin(roll)
  // >>>     ca_p = m.cos(pitch)
  // >>>     sa_p = m.sin(pitch)
  // >>>     ca_y = m.cos(yaw)
  // >>>     sa_y = m.sin(yaw)
  // >>>     #build rotation matrix per axis
  // >>>     rx = np.matrix([[1, 0, 0], [0, ca_r, -sa_r], [0, sa_r, ca_r]])
  // >>>     ry = np.matrix([[ca_p, 0, sa_p], [0, 1, 0], [-sa_p, 0, ca_p]])
  // >>>     rz = np.matrix([[ca_y, -sa_y, 0], [sa_y, ca_y, 0], [0, 0, 1]])
  // >>>     return (rz * ry * rx)
  // >>> rot_mat = rotation_matrix(1.75,2.91,0.38)
  // >>> inertial_position = np.matrix([[15.],[33.],[148.]])
  // >>> print (rot_mat*inertial_position)
  // Tolerance has been empirically found for these testing values.
  const double kRotationTolerance = 1e-8;
  const Rotation dut = Rotation::FromRpy(1.75, 2.91, 0.38);
  const InertialPosition inertial_position = dut.Apply({15., 33., 148.});
  EXPECT_TRUE(test::IsInertialPositionClose(
      inertial_position, InertialPosition{43.93919835, -145.60056097, -9.37141893}, kRotationTolerance));
}

GTEST_TEST(Rotation, DistanceTest) {
  // Results in this test were derived with the following Python script:
  // >>> import math as m
  // >>> import numpy as np
  // >>> def rotation_matrix (roll, pitch ,yaw):
  // >>>     ca_r = m.cos(roll)
  // >>>     sa_r = m.sin(roll)
  // >>>     ca_p = m.cos(pitch)
  // >>>     sa_p = m.sin(pitch)
  // >>>     ca_y = m.cos(yaw)
  // >>>     sa_y = m.sin(yaw)
  // >>>     #build rotation matrix per axis
  // >>>     rx = np.matrix([[1, 0, 0], [0, ca_r, -sa_r], [0, sa_r, ca_r]])
  // >>>     ry = np.matrix([[ca_p, 0, sa_p], [0, 1, 0], [-sa_p, 0, ca_p]])
  // >>>     rz = np.matrix([[ca_y, -sa_y, 0], [sa_y, ca_y, 0], [0, 0, 1]])
  // >>>     return (rz * ry * rx)
  // >>> rot_mat_a = rotation_matrix(1.75,2.91,0.38)
  // >>> rot_mat_b = rotation_matrix(3.1,0.1,2.2)
  // >>> a_s = rot_mat_a * np.matrix([[1.],[0.],[0.]])
  // >>> a_r = rot_mat_a * np.matrix([[0.],[1.],[0.]])
  // >>> a_h = rot_mat_a * np.matrix([[0.],[0.],[1.]])
  // >>> b_s = rot_mat_b * np.matrix([[1.],[0.],[0.]])
  // >>> b_r = rot_mat_b * np.matrix([[0.],[1.],[0.]])
  // >>> b_h = rot_mat_b * np.matrix([[0.],[0.],[1.]])
  // >>> ds = m.acos(np.dot(np.transpose(a_s), b_s))
  // >>> dr = m.acos(np.dot(np.transpose(a_r), b_r))
  // >>> dh = m.acos(np.dot(np.transpose(a_h), b_h))
  // >>> print(m.sqrt(ds**2 + dr**2 + dh**2 ))
  // Tolerance has been empirically found for these testing values.
  const double kRotationTolerance = 1e-11;
  const Rotation dut = Rotation::FromRpy(1.75, 2.91, 0.38);
  EXPECT_NEAR(dut.Distance(Rotation::FromRpy(3.1, 0.1, 2.2)), 2.55482853419, kRotationTolerance);
}

GTEST_TEST(Rotation, ReverseTest) {
  // Results in this test were derived with the following Python script:
  // >>> import math as m
  // >>> import numpy as np
  // >>> def rotation_matrix (roll, pitch ,yaw):
  // >>>     ca_r = m.cos(roll)
  // >>>     sa_r = m.sin(roll)
  // >>>     ca_p = m.cos(pitch)
  // >>>     sa_p = m.sin(pitch)
  // >>>     ca_y = m.cos(yaw)
  // >>>     sa_y = m.sin(yaw)
  // >>>     #build rotation matrix per axis
  // >>>     rx = np.matrix([[1, 0, 0], [0, ca_r, -sa_r], [0, sa_r, ca_r]])
  // >>>     ry = np.matrix([[ca_p, 0, sa_p], [0, 1, 0], [-sa_p, 0, ca_p]])
  // >>>     rz = np.matrix([[ca_y, -sa_y, 0], [sa_y, ca_y, 0], [0, 0, 1]])
  // >>>     return (rz * ry * rx)
  // >>> rot_mat_a = rotation_matrix(0., 0., 0.)
  // >>> rot_mat_b = rotation_matrix(0.,0.,m.pi)
  // >>> rot_mat_reversed = rot_mat_b * rot_mat_a
  // r_s = rot_mat_reversed * np.matrix([[1.],[0.],[0.]]); print(r_s)
  // r_r = rot_mat_reversed * np.matrix([[0.],[1.],[0.]]); print(r_r)
  // r_h = rot_mat_reversed * np.matrix([[0.],[0.],[1.]]); print(r_h)
  // Tolerance has been empirically found for these testing values.
  const double kLinearTolerance = 1e-15;
  const Rotation dut = Rotation::FromRpy(0., 0., 0.).Reverse();
  EXPECT_TRUE(math::test::CompareVectors(math::Vector3(-1., 0., 0.),
                                         dut.quat().TransformVector(math::Vector3(1., 0., 0)), kLinearTolerance));
  EXPECT_TRUE(math::test::CompareVectors(math::Vector3(0., -1., 0.),
                                         dut.quat().TransformVector(math::Vector3(0., 1., 0)), kLinearTolerance));
  EXPECT_TRUE(math::test::CompareVectors(math::Vector3(0., 0., 1.),
                                         dut.quat().TransformVector(math::Vector3(0., 0., 1.)), kLinearTolerance));
}

#undef CHECK_ALL_ROTATION_ACCESSORS

GTEST_TEST(RBoundsTest, DefaultConstructor) {
  const RBounds dut{};
  // Checks correct default value assignment.
  EXPECT_EQ(dut.min(), 0.);
  EXPECT_EQ(dut.max(), 0.);
}

GTEST_TEST(RBoundsTest, ParameterizedConstructor) {
  const double kMin{-5.};
  const double kMax{5.};
  RBounds dut(kMin, kMax);
  // Checks correct value assignment.
  EXPECT_EQ(dut.min(), kMin);
  EXPECT_EQ(dut.max(), kMax);
  // Checks constraints on the constructor.
  EXPECT_THROW(RBounds(kMax, kMax), maliput::common::assertion_error);
  EXPECT_THROW(RBounds(kMin, kMin), maliput::common::assertion_error);
}

GTEST_TEST(RBoundsTest, Setters) {
  const double kMin{-5.};
  const double kMax{5.};
  RBounds dut(kMin, kMax);
  // Set min and max correct values.
  dut.set_min(2. * kMin);
  EXPECT_EQ(dut.min(), 2. * kMin);
  dut.set_max(2. * kMax);
  EXPECT_EQ(dut.max(), 2. * kMax);
  // Checks constraints on the setters.
  EXPECT_THROW(dut.set_min(kMax), maliput::common::assertion_error);
  EXPECT_THROW(dut.set_max(kMin), maliput::common::assertion_error);
}

GTEST_TEST(HBoundsTest, DefaultConstructor) {
  const HBounds dut{};
  // Checks correct default value assignment.
  EXPECT_EQ(dut.min(), 0.);
  EXPECT_EQ(dut.max(), 0.);
}

GTEST_TEST(HBoundsTest, ParameterizedConstructor) {
  const double kMin{-5.};
  const double kMax{5.};
  HBounds dut(kMin, kMax);
  // Checks correct value assignment.
  EXPECT_EQ(dut.min(), kMin);
  EXPECT_EQ(dut.max(), kMax);
  // Checks constraints on the constructor.
  EXPECT_THROW(HBounds(kMax, kMax), maliput::common::assertion_error);
  EXPECT_THROW(HBounds(kMin, kMin), maliput::common::assertion_error);
}

GTEST_TEST(HBoundsTest, Setters) {
  const double kMin{-5.};
  const double kMax{5.};
  HBounds dut(kMin, kMax);
  // Set min and max correct values.
  dut.set_min(2. * kMin);
  EXPECT_EQ(dut.min(), 2. * kMin);
  dut.set_max(2. * kMax);
  EXPECT_EQ(dut.max(), 2. * kMax);
  // Checks constraints on the setters.
  EXPECT_THROW(dut.set_min(kMax), maliput::common::assertion_error);
  EXPECT_THROW(dut.set_max(kMin), maliput::common::assertion_error);
}

}  // namespace
}  // namespace api
}  // namespace maliput
