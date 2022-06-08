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
#include <algorithm>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/common/assertion_error.h"
#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"
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
  const math::Vector3 inertial_to_backend_frame_translation{0., 0., 0.};
  const double kZeroTolerance{0.};
};

class InertialPositionMatcher : public MatcherInterface<const api::InertialPosition&> {
 public:
  InertialPositionMatcher(const api::InertialPosition& inertial_position, double tolerance)
      : inertial_position_(inertial_position), tolerance_(tolerance) {}

  bool MatchAndExplain(const api::InertialPosition& other, MatchResultListener*) const override {
    return maliput::api::test::IsInertialPositionClose(inertial_position_, other, tolerance_);
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is within tolerance: [" << tolerance_ << "] of all x, y, and z coordinates in: [" << inertial_position_
        << "].";
  }

 private:
  const api::InertialPosition inertial_position_;
  const double tolerance_{};
};

Matcher<const api::InertialPosition&> Matches(const api::InertialPosition& inertial_position, double tolerance) {
  return MakeMatcher(new InertialPositionMatcher(inertial_position, tolerance));
}

class LaneMock final : public MockLane {
 public:
  LaneMock(const api::LaneId& id, double distance) : MockLane(id), distance_(distance) {
    ON_CALL(*this, DoToLanePosition(An<const api::InertialPosition&>()))
        .WillByDefault(Invoke(this, &LaneMock::InternalDoToLanePosition));
  }

  MOCK_CONST_METHOD1(DoToLanePosition, api::LanePositionResult(const api::InertialPosition&));

  api::LanePositionResult InternalDoToLanePosition(const api::InertialPosition&) const {
    return api::LanePositionResult{api::LanePosition(4., 5., 6.), api::InertialPosition(10., 11., 12.), distance_};
  }

 private:
  const double distance_{};
};

class RoadGeometryMock final : public MockRoadGeometry {
 public:
  explicit RoadGeometryMock(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance,
                            double scale_length, const math::Vector3& inertial_to_backend_frame_translation)
      : MockRoadGeometry(id, linear_tolerance, angular_tolerance, scale_length, inertial_to_backend_frame_translation) {
  }
  void set_lanes(std::vector<LaneMock*> lanes) { lanes_.assign(lanes.begin(), lanes.end()); }
  std::vector<LaneMock*> get_lanes() { return lanes_; }

 private:
  std::vector<LaneMock*> lanes_;
};

std::unique_ptr<RoadGeometryMock> MakeOneLaneRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance,
                                                          double angular_tolerance, double scale_length,
                                                          const math::Vector3& inertial_to_backend_frame_translation,
                                                          double lane_distance) {
  std::vector<LaneMock*> lanes;
  auto road_geometry = std::make_unique<RoadGeometryMock>(id, linear_tolerance, angular_tolerance, scale_length,
                                                          inertial_to_backend_frame_translation);
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
                                                       double angular_tolerance, double scale_length,
                                                       const math::Vector3& inertial_to_backend_frame_translation) {
  auto road_geometry = std::make_unique<RoadGeometryMock>(id, linear_tolerance, angular_tolerance, scale_length,
                                                          inertial_to_backend_frame_translation);
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
  auto rg = MakeOneLaneRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length,
                                    inertial_to_backend_frame_translation, kDistance);

  std::vector<api::RoadPositionResult> results =
      BruteForceFindRoadPositionsStrategy(rg.get(), api::InertialPosition(1., 2., 3.), kRadius);

  EXPECT_EQ(static_cast<int>(results.size()), 1);
  const api::LanePosition kExpectedLanePosition{4., 5., 6.};
  const api::InertialPosition kExpectedInertialPosition{10., 11., 12.};
  const double kExpectedDistance = 3.;
  const std::vector<LaneMock*> lanes = rg.get()->get_lanes();
  api::LaneId id = results.front().road_position.lane->id();

  EXPECT_TRUE(id == lanes.front()->id());
  EXPECT_TRUE(api::test::IsLanePositionClose(results.front().road_position.pos, kExpectedLanePosition, kZeroTolerance));
  EXPECT_TRUE(
      api::test::IsInertialPositionClose(results.front().nearest_position, kExpectedInertialPosition, kZeroTolerance));
  EXPECT_NEAR(results.front().distance, kExpectedDistance, kZeroTolerance);

  rg = MakeOneLaneRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length,
                               inertial_to_backend_frame_translation, 2. * kDistance);
  results = BruteForceFindRoadPositionsStrategy(rg.get(), api::InertialPosition(1., 2., 3.), kRadius);

  EXPECT_TRUE(results.empty());
}

TEST_F(BruteForceTest, NullRoadGeometry) {
  EXPECT_THROW(BruteForceFindRoadPositionsStrategy(nullptr, api::InertialPosition(0., 0., 0.), 0.),
               common::assertion_error);
}

TEST_F(BruteForceTest, NegativeRadius) {
  std::unique_ptr<MockRoadGeometry> rg =
      MakeFullRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length,
                           inertial_to_backend_frame_translation);
  EXPECT_THROW(BruteForceFindRoadPositionsStrategy(rg.get(), api::InertialPosition(0., 0., 0.), -1.),
               common::assertion_error);
}

TEST_F(BruteForceTest, AllLanesCalled) {
  auto rg = MakeFullRoadGeometry(api::RoadGeometryId("dut"), linear_tolerance, angular_tolerance, scale_length,
                                 inertial_to_backend_frame_translation);

  const std::vector<LaneMock*> lanes = rg.get()->get_lanes();

  for (auto lane : lanes) {
    EXPECT_CALL(*lane, DoToLanePosition(Matches(api::InertialPosition(1., 2., 3.), kZeroTolerance)));
  }

  const std::vector<api::RoadPositionResult> results =
      BruteForceFindRoadPositionsStrategy(rg.get(), api::InertialPosition(1., 2., 3.), 4.);

  EXPECT_EQ(static_cast<int>(results.size()), 3);
  const api::LanePosition kExpectedLanePosition{4., 5., 6.};
  const api::InertialPosition kExpectedInertialPosition{10., 11., 12.};
  const double kExpectedDistance = 3.;
  for (const auto road_position_result : results) {
    EXPECT_TRUE(std::any_of(
        lanes.begin(), lanes.end(),
        [id = road_position_result.road_position.lane->id()](LaneMock* lane) mutable { return id == lane->id(); }));
    EXPECT_TRUE(
        api::test::IsLanePositionClose(road_position_result.road_position.pos, kExpectedLanePosition, kZeroTolerance));
    EXPECT_TRUE(api::test::IsInertialPositionClose(road_position_result.nearest_position, kExpectedInertialPosition,
                                                   kZeroTolerance));
    EXPECT_NEAR(road_position_result.distance, kExpectedDistance, kZeroTolerance);
  }
}

}  // namespace
}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
