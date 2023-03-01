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
/* clang-format off to disable clang-format-includes */
#include "maliput/api/regions.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/regions_test_utilities.h"

namespace maliput {
namespace api {
namespace {

// Tolerance small enough to test possibly actual use cases.
const double kLinearTolerance = 1e-3;
// Negative tolerance used to test SRange::Intersects method.
// Using negative values in this method makes ranges shrink.
const double kNegativeLinearTolerance = -2;
// An excessive negative tolerance used to test SRange::Intersects method.
// Using negative values bigger than the size of one SRange is not allowed
// and an exception will be thrown.
const double kExcessiveNegativeLinearTolerance = -50;
// Arbitrary api::LaneId created for testing purposes.
const LaneId kLaneId1{"Id1"};
const LaneId kLaneId2{"Id2"};
const LaneId kLaneId3{"Id3"};

GTEST_TEST(SRangeTest, DefaultConstructionAndAccessors) {
  const SRange dut;
  EXPECT_EQ(dut.s0(), 0.);
  EXPECT_EQ(dut.s1(), 0.);
}

GTEST_TEST(SRangeTest, NondefaultConstructionAndAccessors) {
  {
    const SRange dut(10., 50.);
    EXPECT_EQ(dut.s0(), 10.);
    EXPECT_EQ(dut.s1(), 50.);
  }
  // Inverted order is allowed and preserved:
  {
    const SRange dut(79., 23.);
    EXPECT_EQ(dut.s0(), 79.);
    EXPECT_EQ(dut.s1(), 23.);
  }
  {
    EXPECT_THROW(SRange(-10., 50.);, common::assertion_error);
    EXPECT_THROW(SRange(10., -50.);, common::assertion_error);
  }
}

GTEST_TEST(SRangeTest, Setters) {
  SRange dut;
  dut.set_s0(26.);
  dut.set_s1(90.);
  EXPECT_EQ(dut.s0(), 26.);
  EXPECT_EQ(dut.s1(), 90.);
  EXPECT_THROW(dut.set_s0(-90.), common::assertion_error);
  EXPECT_THROW(dut.set_s1(-90.), common::assertion_error);
}

GTEST_TEST(SRangeTest, Copying) {
  const SRange source(12., 24.);
  const SRange dut(source);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, source));
}

GTEST_TEST(SRangeTest, Assignment) {
  const SRange source(12., 24.);
  SRange dut;
  dut = source;
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, source));
}

GTEST_TEST(SRangeTest, Size) {
  const SRange dut1(2., 5.);
  EXPECT_EQ(dut1.size(), 3.);
  const SRange dut2(50., 2.);
  EXPECT_EQ(dut2.size(), 48.);
}

GTEST_TEST(SRangeTest, WithS) {
  const SRange dut1(2., 5.);
  EXPECT_TRUE(dut1.WithS());
  const SRange dut2(5., 2.);
  EXPECT_FALSE(dut2.WithS());
  const SRange dut3(6., 6.);
  EXPECT_FALSE(dut3.WithS());
}

GTEST_TEST(SRangeTest, Intersects) {
  const SRange s_range(3., 10.);
  EXPECT_TRUE(s_range.Intersects(SRange{5., 5.}, kLinearTolerance));
  EXPECT_TRUE(s_range.Intersects(SRange{2., 5.}, kLinearTolerance));
  EXPECT_TRUE(s_range.Intersects(SRange{8., 13.}, kLinearTolerance));
  EXPECT_TRUE(s_range.Intersects(SRange{11., 9.}, kLinearTolerance));
  EXPECT_FALSE(s_range.Intersects(SRange{15., 33.}, kLinearTolerance));
  EXPECT_TRUE(s_range.Intersects(SRange{2., 33.}, kNegativeLinearTolerance));
  EXPECT_FALSE(s_range.Intersects(SRange{9., 33.}, kNegativeLinearTolerance));
  EXPECT_THROW(s_range.Intersects(SRange{15., 33.}, kExcessiveNegativeLinearTolerance), common::assertion_error);
  EXPECT_THROW(s_range.Intersects(SRange{-5., 33.}, kLinearTolerance), common::assertion_error);
}

// Holds build configuration for SRangeGetIntersectionTest.
struct SRangeGetIntersectionBuildFlags {
  SRange s_range{};
  SRange intersect{};
  SRange expected{};
  double tolerance{};
  bool expects_nullopt{false};
};

// Tests SRange::GetIntersection method.
class SRangeGetIntersectionTest : public ::testing::TestWithParam<SRangeGetIntersectionBuildFlags> {
 protected:
  void SetUp() override { build_config_ = GetParam(); }

  SRangeGetIntersectionBuildFlags build_config_;
};

std::vector<SRangeGetIntersectionBuildFlags> GetIntersectionTestParameters() {
  return {
      SRangeGetIntersectionBuildFlags{SRange{3., 10.}, SRange{2., 5.}, SRange{3., 5.}, kLinearTolerance, false},
      SRangeGetIntersectionBuildFlags{SRange{10., 3.}, SRange{11., 7.}, SRange{7., 10.}, kLinearTolerance, false},
      SRangeGetIntersectionBuildFlags{SRange{1., 100.}, SRange{49., 50.}, SRange{49., 50.}, kLinearTolerance, false},
      SRangeGetIntersectionBuildFlags{SRange{37., 27.}, SRange{1., 29.}, SRange{27., 29.}, kLinearTolerance, false},
      // There is no intersection.
      SRangeGetIntersectionBuildFlags{SRange{3., 10.}, SRange{11., 75.}, SRange{3., 5.}, kLinearTolerance, true},
  };
}

TEST_P(SRangeGetIntersectionTest, GetIntersection) {
  const std::optional<SRange> dut =
      build_config_.s_range.GetIntersection(build_config_.intersect, build_config_.tolerance);
  if (build_config_.expects_nullopt) {
    EXPECT_FALSE(dut.has_value());
  } else {
    EXPECT_LE(std::fabs(dut.value().s0() - build_config_.expected.s0()), build_config_.tolerance);
    EXPECT_LE(std::fabs(dut.value().s1() - build_config_.expected.s1()), build_config_.tolerance);
  }
}

INSTANTIATE_TEST_CASE_P(SRangeGetIntersectionTestGroup, SRangeGetIntersectionTest,
                        ::testing::ValuesIn(GetIntersectionTestParameters()));

GTEST_TEST(LaneSRangeTest, ConstructionAndAccessors) {
  const LaneSRange dut(kLaneId1, SRange(34., 0.));
  EXPECT_EQ(dut.lane_id(), kLaneId1);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.s_range(), SRange(34., 0.)));

  // Exercise convenient construction via initializer list for s_range.
  EXPECT_NO_THROW(LaneSRange(kLaneId1, {0., 50.}));
}

GTEST_TEST(LaneSRangeTest, Copying) {
  const LaneSRange source(kLaneId1, SRange(20., 30.));
  const LaneSRange dut(source);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, source));
}

GTEST_TEST(LaneSRangeTest, Assignment) {
  const LaneSRange source(kLaneId1, SRange(20., 30.));
  LaneSRange dut(kLaneId2, SRange(40., 99.));
  dut = source;
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, source));
}

GTEST_TEST(LaneSRangeTest, Intersects) {
  const LaneSRange lane_s_range_a(kLaneId1, SRange(20., 30.));
  EXPECT_TRUE(lane_s_range_a.Intersects(LaneSRange{kLaneId1, SRange(25., 25.)}, kLinearTolerance));
  EXPECT_TRUE(lane_s_range_a.Intersects(LaneSRange{kLaneId1, SRange(25., 35.)}, kLinearTolerance));
  EXPECT_TRUE(lane_s_range_a.Intersects(LaneSRange{kLaneId1, SRange(70., 10.)}, kLinearTolerance));
  EXPECT_TRUE(lane_s_range_a.Intersects(LaneSRange{kLaneId1, SRange(15., 25.)}, kLinearTolerance));
  EXPECT_FALSE(lane_s_range_a.Intersects(LaneSRange{kLaneId1, SRange(55., 35.)}, kLinearTolerance));
  EXPECT_FALSE(lane_s_range_a.Intersects(LaneSRange{kLaneId3, SRange(70., 10.)}, kLinearTolerance));
}

GTEST_TEST(LaneSRangeTest, GetIntersection) {
  const LaneSRange lane_s_range_a(kLaneId1, SRange(20., 30.));

  // Different lane id. No intersection is expected
  auto dut = lane_s_range_a.GetIntersection(LaneSRange{kLaneId2, SRange(25., 35.)}, kLinearTolerance);
  EXPECT_FALSE(dut.has_value());

  // Intersection is expected.
  LaneSRange expected_intersection{kLaneId1, SRange(25., 30.)};
  dut = lane_s_range_a.GetIntersection(LaneSRange{kLaneId1, SRange(25., 35.)}, kLinearTolerance);
  ASSERT_TRUE(dut.has_value());
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.value(), expected_intersection));

  // Same lane id, different range. No intersection is expected.
  dut = lane_s_range_a.GetIntersection(LaneSRange{kLaneId1, SRange(35., 40.)}, kLinearTolerance);
  EXPECT_FALSE(dut.has_value());
}

class LaneSRouteTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_.emplace_back(kLaneId1, SRange(0., 10.));
    source_.emplace_back(kLaneId2, SRange(90., 17.));
  }

  std::vector<LaneSRange> source_;
};

TEST_F(LaneSRouteTest, DefaultConstructionAndAccessors) {
  const LaneSRoute dut;
  EXPECT_TRUE(dut.ranges().empty());
}

TEST_F(LaneSRouteTest, NondefaultConstructionAndAccessors) {
  const LaneSRoute dut(source_);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.ranges(), source_));
}

TEST_F(LaneSRouteTest, Copying) {
  const LaneSRoute dut_source(source_);
  const LaneSRoute dut(dut_source);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, dut_source));
}

TEST_F(LaneSRouteTest, Assignment) {
  const LaneSRoute dut_source(source_);
  LaneSRoute dut;
  dut = dut_source;
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, dut_source));
}

TEST_F(LaneSRouteTest, Intersects) {
  EXPECT_TRUE(LaneSRoute(source_).Intersects(LaneSRoute{{{kLaneId1, SRange(5., 5.)}}}, kLinearTolerance));
  EXPECT_TRUE(LaneSRoute(source_).Intersects(LaneSRoute{{{kLaneId1, SRange(5., 35.)}, {kLaneId2, SRange(32., 1.)}}},
                                             kLinearTolerance));
  EXPECT_TRUE(LaneSRoute(source_).Intersects(LaneSRoute{{{kLaneId3, SRange(100., 102.)}, {kLaneId2, SRange(25., 30.)}}},
                                             kLinearTolerance));
  EXPECT_TRUE(LaneSRoute(source_).Intersects(LaneSRoute{{{kLaneId1, SRange(180., 12.)}, {kLaneId2, SRange(1., 50.)}}},
                                             kLinearTolerance));
  EXPECT_FALSE(LaneSRoute(source_).Intersects(LaneSRoute{{{kLaneId1, SRange(180., 12.)}, {kLaneId2, SRange(1., 15.)}}},
                                              kLinearTolerance));
}

// Holds RoadGeometry build configuration.
struct RoadGeometryBuildFlags {
  test::RoadGeometryContiguityBuildFlags rg_contiguity_build_flags{};
  bool expects_throw{false};
};

// Tests IsContiguous function with valid values.
class IsContiguousValidValuesTest : public ::testing::TestWithParam<RoadGeometryBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  RoadGeometryBuildFlags build_flags_;
};

std::vector<RoadGeometryBuildFlags> ContiguityValidValuesTestParameters() {
  // RoadGeometry's angular and linear tolerance.
  // Its value is based on the Cartesian distance between non-contiguous lane endpoints
  // which are created in this test.
  const double linear_tolerance = 1e-3;
  // Its value is based on the angular distance used for non-contiguous lane endpoints
  // which are created in this test.
  const double angular_tolerance = 1e-3;
  return {
      // Contiguous LaneSRoute.
      RoadGeometryBuildFlags{{false, false, linear_tolerance, angular_tolerance}, false},
      // Throws because it does not meet angular tolerance.
      RoadGeometryBuildFlags{{false, true, linear_tolerance, angular_tolerance}, true},
      // Throws because it does not meet linear tolerance.
      RoadGeometryBuildFlags{{true, false, linear_tolerance, angular_tolerance}, true},
      // Throws because it does not meet neither the linear tolerance nor the angular tolerance.
      RoadGeometryBuildFlags{{true, true, linear_tolerance, angular_tolerance}, true},
  };
}

TEST_P(IsContiguousValidValuesTest, ChecksIsContiguousFunctionWithValidValues) {
  std::unique_ptr<RoadGeometry> road_geometry;
  road_geometry = test::CreateMockContiguousRoadGeometry(build_flags_.rg_contiguity_build_flags);
  LaneSRange lane_range_a(LaneId("mock_a"), {0., 10.});
  LaneSRange lane_range_b(LaneId("mock_b"), {0., 10.});
  if (build_flags_.expects_throw) {
    EXPECT_FALSE(IsContiguous(lane_range_a, lane_range_b, road_geometry.get()));
  } else {
    EXPECT_TRUE(IsContiguous(lane_range_a, lane_range_b, road_geometry.get()));
  }
}

INSTANTIATE_TEST_CASE_P(IsContiguousFunctionTestGroup, IsContiguousValidValuesTest,
                        ::testing::ValuesIn(ContiguityValidValuesTestParameters()));

// Holds RoadGeometry build configuration.
struct InvalidArgumentsBuildFlags {
  bool add_null_road_geometry{false};
  bool add_null_lane_a{false};
  bool add_null_lane_b{false};
  bool expects_throw{false};
};

// Tests IsContiguous function with invalid values.
class IsContiguousInvalidArgumentsTest : public ::testing::TestWithParam<InvalidArgumentsBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  InvalidArgumentsBuildFlags build_flags_;
};

std::vector<InvalidArgumentsBuildFlags> ContiguityInvalidValuesTestParameters() {
  return {
      // Contiguous LaneSRoute.
      InvalidArgumentsBuildFlags{false, false, false, false},
      // Throws because of invalid `lane_range_a` pointer.
      InvalidArgumentsBuildFlags{false, true, false, true},
      // Throws because of invalid `lane_range_b` pointer.
      InvalidArgumentsBuildFlags{false, false, true, true},
      // Throws because of invalid `road_geometry` pointer.
      InvalidArgumentsBuildFlags{true, false, false, true},
  };
}

TEST_P(IsContiguousInvalidArgumentsTest, ChecksIsContiguousFunctionWithInvalidValues) {
  std::unique_ptr<RoadGeometry> road_geometry;
  road_geometry = test::CreateMockContiguousRoadGeometry({false, false, 0., 0.});
  const LaneSRange lane_range_a{build_flags_.add_null_lane_a ? LaneId("mock_null") : LaneId("mock_a"), {0., 10.}};
  const LaneSRange lane_range_b{build_flags_.add_null_lane_b ? LaneId("mock_null") : LaneId("mock_b"), {0., 10.}};
  const RoadGeometry* road_geometry_ptr = build_flags_.add_null_road_geometry ? nullptr : road_geometry.get();

  if (build_flags_.expects_throw) {
    EXPECT_THROW(IsContiguous(lane_range_a, lane_range_b, road_geometry_ptr), common::assertion_error);
  } else {
    EXPECT_NO_THROW(IsContiguous(lane_range_a, lane_range_b, road_geometry_ptr));
  }
}

INSTANTIATE_TEST_CASE_P(IsContiguousFunctionTestGroup, IsContiguousInvalidArgumentsTest,
                        ::testing::ValuesIn(ContiguityInvalidValuesTestParameters()));

GTEST_TEST(IsIncluded, BasicUsage) {
  const auto rg = test::CreateTwoLanesRoadGeometry();
  {  // `inertial_pos` does not map any Lane.
    const InertialPosition inertial_pos{80., 100., 45.};
    const LaneSRange lane_s_range_b{LaneId{"lane_b"}, SRange{0., 50.}};
    const std::vector<LaneSRange> region{lane_s_range_b};
    EXPECT_FALSE(IsIncluded(inertial_pos, region, rg.get()));
  }
  const InertialPosition inertial_pos{11.9, 89., 1.};
  {  // `lane_s_range_a` includes `inertial_pos`.
    const LaneSRange lane_s_range_a{LaneId{"lane_a"}, SRange{0., 50.}};
    const LaneSRange lane_s_range_b{LaneId{"lane_b"}, SRange{0., 50.}};
    const std::vector<LaneSRange> region{lane_s_range_a, lane_s_range_b};
    EXPECT_TRUE(IsIncluded(inertial_pos, region, rg.get()));
  }
  {  // `lane_a` includes `inertial_pos` but `lane_s_range_a` does not.
    const LaneSRange lane_s_range_a{LaneId{"lane_a"}, SRange{0., 5.}};
    const std::vector<LaneSRange> region{lane_s_range_a};
    EXPECT_FALSE(IsIncluded(inertial_pos, region, rg.get()));
  }
  {  // `lane_a` includes `inertial_pos` but `lane_s_range_b`'s lane does not.
    const LaneSRange lane_s_range_b{LaneId{"lane_b"}, SRange{0., 50.}};
    const std::vector<LaneSRange> region{lane_s_range_b};
    EXPECT_FALSE(IsIncluded(inertial_pos, region, rg.get()));
  }
}

}  // namespace
}  // namespace api
}  // namespace maliput
