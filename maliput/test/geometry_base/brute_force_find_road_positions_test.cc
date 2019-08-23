/* clang-format off to disable clang-format-includes */
#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "maliput/test_utilities/mock_geometry.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/api/lane.h"

using ::testing::Expectation;
using ::testing::ExpectationSet;
using ::testing::Matcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

namespace maliput {
namespace geometry_base {
namespace test {
namespace {

class GeoPositionMatcher : public MatcherInterface<const api::GeoPosition&> {
 public:
  GeoPositionMatcher(const api::GeoPosition& geo_position, double tolerance)
      : geo_position_(geo_position), tolerance_(tolerance) {}

  bool MatchAndExplain(const api::GeoPosition& other, MatchResultListener*) const override {
   //TODO implement GeoPosition comparison
   return false;
  }
  
  void DescribeTo(std::ostream* os) const override {
   //TODO implement description
  }

 private:
  const api::GeoPosition geo_position_;
  const double tolerance_{};
};

Matcher<const api::GeoPosition&> Matches(const api::GeoPosition& geo_position, double tolerance) {
  return MakeMatcher(new GeoPositionMatcher(geo_position, tolerance));
}

class LaneMock final : public MockLane {
 public:
  explicit LaneMock(const api::LaneId& id) : MockLane(id) {}
 
 private: 
  api::LanePosition DoToLanePosition(const api::GeoPosition& geo_pos, api::GeoPosition* nearest_point, double* distance) const override { 
   return api::LanePosition(0, 0, 0);
  }
  double distance;
};

std::unique_ptr<MockRoadGeometry> CreateFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) {
 //LaneMock l1
 //LaneMock lane0 = std::make_unique<LaneMock>(api::LaneId("lane0"));	
 auto lane0 = std::make_unique<LaneMock>(api::LaneId("lane0"));	
 auto lane1 = std::make_unique<LaneMock>(api::LaneId("lane1"));	
 auto lane2 = std::make_unique<LaneMock>(api::LaneId("lane2"));	
 
 auto segment0 = std::make_unique<MockSegment>(api::SegmentId("segment0"));
 auto segment1 = std::make_unique<MockSegment>(api::SegmentId("segment1"));
 
 segment0->AddLane(std::move(lane0));
 segment1->AddLane(std::move(lane1));
 segment1->AddLane(std::move(lane2));

 auto junction0 = std::make_unique<MockJunction>(api::JunctionId("junction0"));
 auto junction1 = std::make_unique<MockJunction>(api::JunctionId("junction1"));

 junction0->AddSegment(std::move(segment0)); 
 junction1->AddSegment(std::move(segment1)); 

 auto road_geometry = std::make_unique<MockRoadGeometry>(id, linear_tolerance, angular_tolerance, scale_length);

 road_geometry->AddJunction(std::move(junction0));
 road_geometry->AddJunction(std::move(junction1));

 return road_geometry;
}

/*
class SegmentMock : public geometry_base::Segment {
 public:
  explicit SegmentMock(const api::SegmentId& id) {
   // TODO assign segment_ here
   //segment_ = std::make_unique<geometry_base::Segment>(id);
  }
 private:
  std::unique_ptr<Segment> segment_;
};

MockRoadGeometry CreateFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) {

}
*/
GTEST_TEST(BruteForceTest, NullRoadGeometry) {
   EXPECT_THROW(BruteForceFindRoadPositionsStrategy(nullptr, api::GeoPosition(0., 0., 0.), 0.), std::exception);
}

GTEST_TEST(BruteForceTest, NegativeRadius) {
   std::unique_ptr<MockRoadGeometry> rg = CreateFullRoadGeometry(api::RoadGeometryId("dut"), 1., 1., 1.);
   MockRoadGeometry* rg2 = rg.get();
   EXPECT_THROW(BruteForceFindRoadPositionsStrategy(rg2, api::GeoPosition(0., 0., 0.), -1.), std::exception);
}

GTEST_TEST(BruteForceTest, VerifyArgs) {
  auto local_rg = CreateFullRoadGeometry(api::RoadGeometryId("dut"), 1., 1., 1.);
  double distance{};
  api::GeoPosition nearest_position;
  api::RoadGeometry* rg = local_rg.get();

  ExpectationSet prebuild_expectations;
  /*
  prebuild_expectations += EXPECT_CALL(
      *rg,
      DoToLanePosition(Matches(api::GeoPosition(1, 2, 3)),
                       &nearest_position,
		       &distance));
  */
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
