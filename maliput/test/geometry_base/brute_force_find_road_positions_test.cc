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
  explicit LaneMock(const api::LaneId& id) : MockLane(id) {
    ON_CALL(*this, DoToLanePosition(An<const api::GeoPosition&>(), An<api::GeoPosition*>(), An<double*>()))
        .WillByDefault(Invoke(this, &LaneMock::InternalDoToLanePosition));
  }

  MOCK_CONST_METHOD3(DoToLanePosition, api::LanePosition(const api::GeoPosition&, api::GeoPosition*, double*));

  api::LanePosition InternalDoToLanePosition(const api::GeoPosition&, api::GeoPosition* nearest_pos,
                                             double* distance) const {
    *nearest_pos = api::GeoPosition(10., 11., 12.);
    *distance = 3.;
    return api::LanePosition(4., 5., 6.);
  }
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

std::unique_ptr<RoadGeometryMock> MakeFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance,
                                                       double angular_tolerance, double scale_length) {
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
    EXPECT_CALL(*lane, DoToLanePosition(Matches(api::GeoPosition(1., 2., 3.), kZeroTolerance), _, _));
  }

  const std::vector<api::RoadPositionResult> results =
      BruteForceFindRoadPositionsStrategy(rg.get(), api::GeoPosition(1., 2., 3.), 4.);

  EXPECT_EQ(results.size(), 3);
  const api::LanePosition kExpectedLanePosition{4., 5., 6.};
  const api::GeoPosition kExpectedGeoPosition{10., 11., 12.};
  const double kExpectedDistance = 3.;
  for (const auto road_position_result : results) {
    EXPECT_TRUE(std::any_of(
        lanes.begin(), lanes.end(),
        [id = road_position_result.road_position.lane->id()](LaneMock* lane) mutable { return id == lane->id(); }));
    api::test::IsLanePositionClose(road_position_result.road_position.pos, kExpectedLanePosition, kZeroTolerance);
    api::test::IsGeoPositionClose(road_position_result.nearest_position, kExpectedGeoPosition, kZeroTolerance);
    EXPECT_NEAR(road_position_result.distance, kExpectedDistance, kZeroTolerance);
  }
}

}  // namespace
}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
