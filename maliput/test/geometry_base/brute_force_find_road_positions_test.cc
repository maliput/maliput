#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"

#include <algorithm>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/maliput_types_compare.h"
#include "maliput/test_utilities/mock_geometry.h"
#include "maliput/test_utilities/rules_test_utilities.h"

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

const double kRadius{3.};
const double kDistance{3.};

class BruteForceTest : public ::testing::Test {
 protected:
  const double linear_tolerance{1.};
  const double angular_tolerance{1.};
  const double scale_length{1.};
  const double kZeroTolerance{0.};
};

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
  LaneMock(const api::LaneId& id, double distance) : MockLane(id), distance_(distance) {
    ON_CALL(*this, DoToLanePosition(An<const api::GeoPosition&>()))
        .WillByDefault(Invoke(this, &LaneMock::InternalDoToLanePosition));
  }

  MOCK_CONST_METHOD1(DoToLanePosition, api::LanePositionResult(const api::GeoPosition&));

  api::LanePositionResult InternalDoToLanePosition(const api::GeoPosition&) const {
    return api::LanePositionResult{api::LanePosition(4., 5., 6.), api::GeoPosition(10., 11., 12.), distance_};
  }

 private:
  const double distance_{};
};

class RoadGeometryMock final : public MockRoadGeometry {
 public:
  explicit RoadGeometryMock(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance,
                            double scale_length)
      : MockRoadGeometry(id, linear_tolerance, angular_tolerance, scale_length) {}
  void set_lanes(std::vector<LaneMock*> lanes) { lanes_.assign(lanes.begin(), lanes.end()); }
  std::vector<LaneMock*> get_lanes() { return lanes_; }

 private:
  std::vector<LaneMock*> lanes_;
};

std::unique_ptr<RoadGeometryMock> MakeOneLaneRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance,
                                                          double angular_tolerance, double scale_length,
                                                          double lane_distance) {
  std::vector<LaneMock*> lanes;
  auto road_geometry = std::make_unique<RoadGeometryMock>(id, linear_tolerance, angular_tolerance, scale_length);
  auto lane0 = std::make_unique<LaneMock>(api::LaneId("lane0"), lane_distance);
  auto segment0 = std::make_unique<MockSegment>(api::SegmentId("segment0"));
  auto junction0 = std::make_unique<MockJunction>(api::JunctionId("junction0"));

  lanes.push_back(segment0->AddLane(std::move(lane0)));
  junction0->AddSegment(std::move(segment0));
  road_geometry->AddJunction(std::move(junction0));

  road_geometry->set_lanes(lanes);

  return road_geometry;
}

std::unique_ptr<RoadGeometryMock> MakeFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance,
                                                       double angular_tolerance, double scale_length) {
  auto road_geometry = std::make_unique<RoadGeometryMock>(id, linear_tolerance, angular_tolerance, scale_length);
  std::vector<LaneMock*> lanes;

  auto lane0 = std::make_unique<LaneMock>(api::LaneId("lane0"), kDistance);
  auto lane1 = std::make_unique<LaneMock>(api::LaneId("lane1"), kDistance);
  auto lane2 = std::make_unique<LaneMock>(api::LaneId("lane2"), kDistance);

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

TEST_F(BruteForceTest, LaneInAndOutRadius) {
  auto rg =
      MakeOneLaneRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length, kDistance);

  std::vector<api::RoadPositionResult> results =
      BruteForceFindRoadPositionsStrategy(rg.get(), api::GeoPosition(1., 2., 3.), kRadius);

  EXPECT_EQ(static_cast<int>(results.size()), 1);
  const api::LanePosition kExpectedLanePosition{4., 5., 6.};
  const api::GeoPosition kExpectedGeoPosition{10., 11., 12.};
  const double kExpectedDistance = 3.;
  const std::vector<LaneMock*> lanes = rg.get()->get_lanes();
  api::LaneId id = results.front().road_position.lane->id();

  EXPECT_TRUE(id == lanes.front()->id());
  EXPECT_TRUE(api::test::IsLanePositionClose(results.front().road_position.pos, kExpectedLanePosition, kZeroTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(results.front().nearest_position, kExpectedGeoPosition, kZeroTolerance));
  EXPECT_NEAR(results.front().distance, kExpectedDistance, kZeroTolerance);

  rg = MakeOneLaneRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length,
                               2. * kDistance);
  results = BruteForceFindRoadPositionsStrategy(rg.get(), api::GeoPosition(1., 2., 3.), kRadius);

  EXPECT_TRUE(results.empty());
}

TEST_F(BruteForceTest, NullRoadGeometry) {
  EXPECT_THROW(BruteForceFindRoadPositionsStrategy(nullptr, api::GeoPosition(0., 0., 0.), 0.), common::assertion_error);
}

TEST_F(BruteForceTest, NegativeRadius) {
  std::unique_ptr<MockRoadGeometry> rg =
      MakeFullRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length);
  EXPECT_THROW(BruteForceFindRoadPositionsStrategy(rg.get(), api::GeoPosition(0., 0., 0.), -1.),
               common::assertion_error);
}

TEST_F(BruteForceTest, AllLanesCalled) {
  auto rg = MakeFullRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length);

  const std::vector<LaneMock*> lanes = rg.get()->get_lanes();

  for (auto lane : lanes) {
    EXPECT_CALL(*lane, DoToLanePosition(Matches(api::GeoPosition(1., 2., 3.), kZeroTolerance)));
  }

  const std::vector<api::RoadPositionResult> results =
      BruteForceFindRoadPositionsStrategy(rg.get(), api::GeoPosition(1., 2., 3.), 4.);

  EXPECT_EQ(static_cast<int>(results.size()), 3);
  const api::LanePosition kExpectedLanePosition{4., 5., 6.};
  const api::GeoPosition kExpectedGeoPosition{10., 11., 12.};
  const double kExpectedDistance = 3.;
  for (const auto road_position_result : results) {
    EXPECT_TRUE(std::any_of(
        lanes.begin(), lanes.end(),
        [id = road_position_result.road_position.lane->id()](LaneMock* lane) mutable { return id == lane->id(); }));
    EXPECT_TRUE(
        api::test::IsLanePositionClose(road_position_result.road_position.pos, kExpectedLanePosition, kZeroTolerance));
    EXPECT_TRUE(
        api::test::IsGeoPositionClose(road_position_result.nearest_position, kExpectedGeoPosition, kZeroTolerance));
    EXPECT_NEAR(road_position_result.distance, kExpectedDistance, kZeroTolerance);
  }
}

}  // namespace
}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
