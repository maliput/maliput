/* clang-format off to disable clang-format-includes */
#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "maliput/test_utilities/mock_geometry.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/api/lane.h"


namespace maliput {
namespace geometry_base {
namespace test {
namespace {

class LaneMock final : public maliput::api::Lane {
 public:
  LaneMock() {}
  
 private: 
  maliput::api::LanePosition DoToLanePosition(const maliput::api::GeoPosition& geo_pos, maliput::api::GeoPosition* nearest_point, double* distance) const override { 
    return maliput::api::LanePosition(0, 0, 0);
  } 
};

/*
MockRoadGeometry CreateFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) {

}
*/
GTEST_TEST(BruteForceTest, NullRoadGeometry) {
  // const SRange dut;
  // EXPECT_THROW();
  // EXPECT_EQ(dut.s0(), 0.);
  // EXPECT_EQ(dut.s1(), 0.);
}

/*
GTEST_TEST(BruteForceTest, NondefaultConstructionAndAccessors) {
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

GTEST_TEST(BruteForceTest, Setters) {
  SRange dut;
  dut.set_s0(26.);
  dut.set_s1(-90.);
  EXPECT_EQ(dut.s0(), 26.);
  EXPECT_EQ(dut.s1(), -90.);
}

GTEST_TEST(BruteForceTest, Copying) {
  const SRange source(12., 24.);
  const SRange dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(BruteForceTest, Assignment) {
  const SRange source(12., 24.);
  SRange dut;
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(BruteForceTest, ConstructionAndAccessors) {
  LaneSRange dut(LaneId("dut"), SRange(34., 0.));
  EXPECT_EQ(dut.lane_id(), LaneId("dut"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.s_range(), SRange(34., 0.)));

  // Exercise convenient construction via initializer list for s_range.
  EXPECT_NO_THROW(LaneSRange(LaneId("dut"), {0., 50.}));
}

GTEST_TEST(BruteForceTest, Copying) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  const LaneSRange dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(LaneSRangeTest, Assignment) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  LaneSRange dut(LaneId("yyy"), SRange(40., 99.));  // e.g., "something else"
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}
*/
}  // namespace
}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
