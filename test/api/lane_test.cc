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
#include <exception>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/intersection.h"
#include "maliput/api/road_network.h"
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
  const math::Vector3 inertial_to_backend_frame_translation{0., 0., 0.};
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

  double InternalLength() const { return kLaneLength; }
};

class RoadGeometryMock final : public geometry_base::test::MockRoadGeometry {
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

std::unique_ptr<RoadGeometryMock> MakeFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance,
                                                       double angular_tolerance, double scale_length,
                                                       const math::Vector3& inertial_to_backend_frame_translation) {
  auto road_geometry = std::make_unique<RoadGeometryMock>(id, linear_tolerance, angular_tolerance, scale_length,
                                                          inertial_to_backend_frame_translation);
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
  const LanePosition false_lane_position = LanePosition(s + kLaneLength + linear_tolerance, r, h);

  auto rg = MakeFullRoadGeometry(api::RoadGeometryId("mock_road_geometry"), linear_tolerance, angular_tolerance,
                                 scale_length, inertial_to_backend_frame_translation);

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
