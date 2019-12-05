#include "maliput/api/road_network.h"

#include <exception>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/intersection.h"
#include "maliput/geometry_base/road_geometry.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/mock_geometry.h"

using ::testing::An;
using ::testing::Invoke;

namespace maliput {
namespace api {
namespace test {
namespace {

const double kLaneLength{10.};
const double kOneTolerance{1.};

class LaneTest : public ::testing::Test {
 protected:
  const double linear_tolerance{kOneTolerance};
  const double angular_tolerance{1.};
  const double scale_length{1.};
  const double s{1};
  const double r{2};
  const double h{3};
};

class LaneMock final : public geometry_base::test::MockLane {
 public:
  LaneMock(const api::LaneId& id) : MockLane(id) {
    ON_CALL(*this, do_segment_bounds(An<double>())).WillByDefault(Invoke(this, &LaneMock::InternalSegmentBounds));
    ON_CALL(*this, do_elevation_bounds(An<double>(), An<double>()))
        .WillByDefault(Invoke(this, &LaneMock::InternalElevationBounds));
    ON_CALL(*this, do_length()).WillByDefault(Invoke(this, &LaneMock::InternalLength));
  }

  MOCK_CONST_METHOD1(do_segment_bounds, RBounds(double));
  MOCK_CONST_METHOD2(do_elevation_bounds, HBounds(double, double));
  MOCK_CONST_METHOD0(do_length, double());

  RBounds InternalSegmentBounds(double s) const { return RBounds(-3, 3); }

  HBounds InternalElevationBounds(double s, double r) const { return HBounds(-5, 5); }

  double InternalLength() const { return lane_length; }
};

class RoadGeometryMock final : public geometry_base::test::MockRoadGeometry {
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

  auto segment0 = std::make_unique<geometry_base::test::MockSegment>(api::SegmentId("segment0"));
  auto segment1 = std::make_unique<geometry_base::test::MockSegment>(api::SegmentId("segment1"));

  lanes.push_back(segment0->AddLane(std::move(lane0)));
  lanes.push_back(segment1->AddLane(std::move(lane1)));
  lanes.push_back(segment1->AddLane(std::move(lane2)));

  road_geometry->set_lanes(lanes);

  auto junction0 = std::make_unique<geometry_base::test::MockJunction>(api::JunctionId("junction0"));
  auto junction1 = std::make_unique<geometry_base::test::MockJunction>(api::JunctionId("junction1"));

  junction0->AddSegment(std::move(segment0));
  junction1->AddSegment(std::move(segment1));

  road_geometry->AddJunction(std::move(junction0));
  road_geometry->AddJunction(std::move(junction1));

  return road_geometry;
}

TEST_F(LaneTest, Contains) {
  const LanePosition true_lane_position = LanePosition(s, r, h);
  const LanePosition false_lane_position = LanePosition(s + lane_length + linear_tolerance, r, h);

  auto rg = MakeFullRoadGeometry(api::RoadGeometryId("mock_road_geometry"), linear_tolerance, angular_tolerance,
                                 scale_length);

  const std::vector<LaneMock*> lanes = rg.get()->get_lanes();

  for (auto lane : lanes) {
    EXPECT_CALL(*lane, do_segment_bounds(true_lane_position.s()));
    EXPECT_CALL(*lane, do_elevation_bounds(true_lane_position.s(), true_lane_position.r()));
    EXPECT_CALL(*lane, do_length());
    EXPECT_TRUE(lane->Contains(true_lane_position));
    EXPECT_CALL(*lane, do_segment_bounds(false_lane_position.s()));
    EXPECT_CALL(*lane, do_elevation_bounds(false_lane_position.s(), false_lane_position.r()));
    EXPECT_CALL(*lane, do_length());
    EXPECT_FALSE(lane->Contains(false_lane_position));
  }
}

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
