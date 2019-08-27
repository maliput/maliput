/* clang-format off to disable clang-format-includes */
#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "maliput/test_utilities/mock_geometry.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/test_utilities/maliput_types_compare.h"
#include "maliput/api/lane.h"

using ::testing::_;
using ::testing::An;
using ::testing::Expectation;
using ::testing::ExpectationSet;
using ::testing::Invoke;
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
    return maliput::api::test::IsGeoPositionClose(geo_position_, other, tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
   *os << "is within tolerance: [" << tolerance_ << "] of all x, y, and z coordinates in: [" << geo_position_ << "].";
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
  explicit LaneMock(const api::LaneId& id) : MockLane(id) {
    ON_CALL(*this, DoToLanePosition(An<const api::GeoPosition&>(), An<api::GeoPosition*>(), An<double*>())).WillByDefault(Invoke(this, &LaneMock::InternalDoToLanePosition));
  }

  MOCK_CONST_METHOD3(DoToLanePosition, api::LanePosition(const api::GeoPosition&, api::GeoPosition*, double*));

  api::LanePosition InternalDoToLanePosition(const api::GeoPosition&, api::GeoPosition*, double*) const {
    return api::LanePosition(4., 5., 6.);
  }
};

class RoadGeometryMock final : public MockRoadGeometry {
 public:
  explicit RoadGeometryMock(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) : MockRoadGeometry(id, linear_tolerance, angular_tolerance, scale_length) {}
  void set_lanes(std::vector<LaneMock*> lanes) { lanes_.assign(lanes.begin(), lanes.end()); }
  std::vector<LaneMock*> get_lanes() { return lanes_; }

 private:
  std::vector<LaneMock*> lanes_;
};

std::unique_ptr<RoadGeometryMock> CreateFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) {
 auto road_geometry = std::make_unique<RoadGeometryMock>(id, linear_tolerance, angular_tolerance, scale_length);
 std::vector<LaneMock*> lanes;
 
 auto lane0 = std::make_unique<LaneMock>(api::LaneId("lane0"));
 auto lane1 = std::make_unique<LaneMock>(api::LaneId("lane1"));
 auto lane2 = std::make_unique<LaneMock>(api::LaneId("lane2"));

 auto segment0 = std::make_unique<MockSegment>(api::SegmentId("segment0"));
 auto segment1 = std::make_unique<MockSegment>(api::SegmentId("segment1"));

 lanes.push_back(segment0->AddLane(std::move(lane0)));
 lanes.push_back(segment1->AddLane(std::move(lane1)));
 lanes.push_back(segment1->AddLane(std::move(lane2)));

 road_geometry->set_lanes(lanes);

 auto junction0 = std::make_unique<MockJunction>(api::JunctionId("junction0"));
 auto junction1 = std::make_unique<MockJunction>(api::JunctionId("junction1"));

 junction0->AddSegment(std::move(segment0));
 junction1->AddSegment(std::move(segment1));

 road_geometry->AddJunction(std::move(junction0));
 road_geometry->AddJunction(std::move(junction1));

 return road_geometry;
}

GTEST_TEST(BruteForceTest, NullRoadGeometry) {
   EXPECT_THROW(BruteForceFindRoadPositionsStrategy(nullptr, api::GeoPosition(0., 0., 0.), 0.), std::exception);
}

GTEST_TEST(BruteForceTest, NegativeRadius) {
   std::unique_ptr<MockRoadGeometry> rg = CreateFullRoadGeometry(api::RoadGeometryId("dut"), 1., 1., 1.);
   MockRoadGeometry* rg2 = rg.get();
   EXPECT_THROW(BruteForceFindRoadPositionsStrategy(rg2, api::GeoPosition(0., 0., 0.), -1.), std::exception);
}

GTEST_TEST(BruteForceTest, AllLanesCalled) {
  double tolerance{0.01};
  auto local_rg = CreateFullRoadGeometry(api::RoadGeometryId("dut"), 1., 1., 1.);

  RoadGeometryMock* rg = local_rg.get();

  std::vector<LaneMock*> lanes = rg->get_lanes();

  for(auto lane : lanes){
    EXPECT_CALL(*lane, DoToLanePosition(Matches(api::GeoPosition(1., 2., 3.), tolerance), _, _));
  }

  std::vector<api::RoadPositionResult> results =
      BruteForceFindRoadPositionsStrategy(rg, api::GeoPosition(1., 2., 3.), 1.);

  EXPECT_EQ(results.size(), 3);
  for (const auto road_position_result : results) {
    EXPECT_EQ(road_position_result.road_position.pos.s(), 4.);
    EXPECT_EQ(road_position_result.road_position.pos.r(), 5.);
    EXPECT_EQ(road_position_result.road_position.pos.h(), 6.);
  }
}

}  // namespace
}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
