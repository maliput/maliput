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
    SRange dut(79., 23.);
    EXPECT_EQ(dut.s0(), 79.);
    EXPECT_EQ(dut.s1(), 23.);
  }
}

GTEST_TEST(SRangeTest, Setters) {
  SRange dut;
  dut.set_s0(26.);
  dut.set_s1(-90.);
  EXPECT_EQ(dut.s0(), 26.);
  EXPECT_EQ(dut.s1(), -90.);
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
  const SRange dut2(5., -2.);
  EXPECT_EQ(dut2.size(), 7.);
}

GTEST_TEST(SRangeTest, WithS) {
  const SRange dut1(2., 5.);
  EXPECT_TRUE(dut1.WithS());
  const SRange dut2(5., -2.);
  EXPECT_FALSE(dut2.WithS());
  const SRange dut3(6., 6.);
  EXPECT_FALSE(dut3.WithS());
}

GTEST_TEST(LaneSRangeTest, ConstructionAndAccessors) {
  LaneSRange dut(LaneId("dut"), SRange(34., 0.));
  EXPECT_EQ(dut.lane_id(), LaneId("dut"));
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.s_range(), SRange(34., 0.)));

  // Exercise convenient construction via initializer list for s_range.
  EXPECT_NO_THROW(LaneSRange(LaneId("dut"), {0., 50.}));
}

GTEST_TEST(LaneSRangeTest, Copying) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  const LaneSRange dut(source);
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, source));
}

GTEST_TEST(LaneSRangeTest, Assignment) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  LaneSRange dut(LaneId("yyy"), SRange(40., 99.));  // e.g., "something else"
  dut = source;
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut, source));
}

class LaneSRouteTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_.emplace_back(LaneId("id1"), SRange(0., 10.));
    source_.emplace_back(LaneId("id2"), SRange(90., 17.));
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
struct AlterRoadGeometryBuildFlags {
  bool add_null_road_geometry{false};
  bool add_null_lane_a{false};
  bool add_null_lane_b{false};
  bool expects_throw{false};
};

// Tests IsContiguous function with invalid values.
class IsContiguousInvalidValuesTest : public ::testing::TestWithParam<AlterRoadGeometryBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  AlterRoadGeometryBuildFlags build_flags_;
};

std::vector<AlterRoadGeometryBuildFlags> ContiguityInvalidValuesTestParameters() {
  return {
      // Contiguous LaneSRoute.
      AlterRoadGeometryBuildFlags{false, false, false, false},
      // Throws because of invalid `lane_range_a` pointer.
      AlterRoadGeometryBuildFlags{false, false, true, true},
      // Throws because of invalid `lane_range_b` pointer.
      AlterRoadGeometryBuildFlags{false, true, false, true},
      // Throws because of invalid `lane_range_a` and `lane_range_b` pointers.
      AlterRoadGeometryBuildFlags{false, true, true, true},
      // Throws because of invalid `road_geometry` pointer.
      AlterRoadGeometryBuildFlags{true, false, false, true},
      // Throws because of invalid `road_geometry` and `lane_range_a` pointers.
      AlterRoadGeometryBuildFlags{true, false, true, true},
      // Throws because of invalid `road_geometry` and `lane_range_b` pointers.
      AlterRoadGeometryBuildFlags{true, true, false, true},
      // Throws because of invalid `road_geometry`, `lane_range_a` and `lane_range_b` pointers.
      AlterRoadGeometryBuildFlags{true, true, true, true},
  };
}

TEST_P(IsContiguousInvalidValuesTest, ChecksIsContiguousFunctionWithInvalidValues) {
  std::unique_ptr<RoadGeometry> road_geometry;
  road_geometry = test::CreateMockContiguousRoadGeometry({false, false, 0., 0.});
  LaneSRange lane_range_b(LaneId("mock_b"), {0., 10.});
  LaneSRange lane_range_a(LaneId("mock_a"), {0., 10.});

  if (build_flags_.add_null_road_geometry) {
    road_geometry = nullptr;
  }
  if (build_flags_.add_null_lane_a) {
    lane_range_a = LaneSRange{LaneId("mock_null"), {0., 10.}};
  }
  if (build_flags_.add_null_lane_b) {
    lane_range_b = LaneSRange{LaneId("mock_null"), {0., 10.}};
  }
  if ((build_flags_.add_null_road_geometry || build_flags_.add_null_lane_a || build_flags_.add_null_lane_b) &&
      build_flags_.expects_throw) {
    EXPECT_THROW(IsContiguous(lane_range_a, lane_range_b, road_geometry.get()), common::assertion_error);
  } else {
    EXPECT_NO_THROW(IsContiguous(lane_range_a, lane_range_b, road_geometry.get()));
  }
}

INSTANTIATE_TEST_CASE_P(IsContiguousFunctionTestGroup, IsContiguousInvalidValuesTest,
                        ::testing::ValuesIn(ContiguityInvalidValuesTestParameters()));

}  // namespace
}  // namespace api
}  // namespace maliput
