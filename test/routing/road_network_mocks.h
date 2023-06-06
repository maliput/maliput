// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
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
#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <maliput/api/branch_point.h>
#include <maliput/api/intersection_book.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/regions.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/phase_provider.h>
#include <maliput/api/rules/phase_ring_book.h>
#include <maliput/api/rules/range_value_rule_state_provider.h>
#include <maliput/api/rules/right_of_way_rule_state_provider.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/api/rules/rule_registry.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/api/segment.h>

namespace maliput {
namespace routing {
namespace test {

/// @brief Google mock maliput::api::RoadGeometry.
class RoadGeometryMock final : public maliput::api::RoadGeometry {
 public:
  MOCK_METHOD(maliput::api::RoadGeometryId, do_id, (), (const));
  MOCK_METHOD(int, do_num_junctions, (), (const));
  MOCK_METHOD(const maliput::api::Junction*, do_junction, (int), (const));
  MOCK_METHOD(int, do_num_branch_points, (), (const));
  MOCK_METHOD(const maliput::api::BranchPoint*, do_branch_point, (int), (const));
  MOCK_METHOD(const maliput::api::RoadGeometry::IdIndex&, DoById, (), (const));
  MOCK_METHOD(maliput::api::RoadPositionResult, DoToRoadPosition,
              (const maliput::api::InertialPosition&, const std::optional<maliput::api::RoadPosition>&), (const));
  MOCK_METHOD(std::vector<maliput::api::RoadPositionResult>, DoFindRoadPositions,
              (const maliput::api::InertialPosition&, double), (const));
  MOCK_METHOD(double, do_linear_tolerance, (), (const));
  MOCK_METHOD(double, do_angular_tolerance, (), (const));
  MOCK_METHOD(double, do_scale_length, (), (const));
  MOCK_METHOD(std::vector<maliput::api::InertialPosition>, DoSampleAheadWaypoints,
              (const maliput::api::LaneSRoute&, double), (const));
  MOCK_METHOD(maliput::math::Vector3, do_inertial_to_backend_frame_translation, (), (const));
};

/// @brief Google mock maliput::api::Lane.
class LaneMock final : public maliput::api::Lane {
 public:
  MOCK_METHOD(maliput::api::LaneId, do_id, (), (const));
  MOCK_METHOD(int, do_index, (), (const));
  MOCK_METHOD(const maliput::api::Segment*, do_segment, (), (const));
  MOCK_METHOD(const maliput::api::Lane*, do_to_left, (), (const));
  MOCK_METHOD(const maliput::api::Lane*, do_to_right, (), (const));
  MOCK_METHOD(double, do_length, (), (const));
  MOCK_METHOD(const maliput::api::BranchPoint*, DoGetBranchPoint, (const maliput::api::LaneEnd::Which), (const));
  MOCK_METHOD(std::optional<maliput::api::LaneEnd>, DoGetDefaultBranch, (const maliput::api::LaneEnd::Which), (const));
  MOCK_METHOD(maliput::api::RBounds, do_lane_bounds, (double), (const));
  MOCK_METHOD(maliput::api::RBounds, do_segment_bounds, (double), (const));
  MOCK_METHOD(maliput::api::HBounds, do_elevation_bounds, (double, double), (const));
  MOCK_METHOD(maliput::api::InertialPosition, DoToInertialPosition, (const maliput::api::LanePosition&), (const));
  MOCK_METHOD(maliput::api::Rotation, DoGetOrientation, (const maliput::api::LanePosition&), (const));
  MOCK_METHOD(maliput::api::LanePosition, DoEvalMotionDerivatives,
              (const maliput::api::LanePosition&, const maliput::api::IsoLaneVelocity&), (const));
  MOCK_METHOD(maliput::api::LanePositionResult, DoToLanePosition, (const maliput::api::InertialPosition&), (const));
  MOCK_METHOD(maliput::api::LanePositionResult, DoToSegmentPosition, (const maliput::api::InertialPosition&), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetConfluentBranches, (const maliput::api::LaneEnd::Which), (const));
  MOCK_METHOD(const maliput::api::LaneEndSet*, DoGetOngoingBranches, (const maliput::api::LaneEnd::Which), (const));
};

/// @brief Google mock maliput::api::RoadGeometry::IdIndex.
class IdIndexMock final : public maliput::api::RoadGeometry::IdIndex {
 public:
  MOCK_METHOD(const maliput::api::Lane*, DoGetLane, (const maliput::api::LaneId&), (const));
  MOCK_METHOD((const std::unordered_map<maliput::api::LaneId, const maliput::api::Lane*>&), DoGetLanes, (), (const));
  MOCK_METHOD(const maliput::api::Segment*, DoGetSegment, (const maliput::api::SegmentId&), (const));
  MOCK_METHOD(const maliput::api::Junction*, DoGetJunction, (const maliput::api::JunctionId&), (const));
  MOCK_METHOD(const maliput::api::BranchPoint*, DoGetBranchPoint, (const maliput::api::BranchPointId&), (const));
};

// @brief Google mock maliput::api::rules::RoadRulebook.
class RoadRulebookMock final : public maliput::api::rules::RoadRulebook {
 public:
  MOCK_METHOD(maliput::api::rules::RoadRulebook::QueryResults, DoFindRules,
              (const std::vector<maliput::api::LaneSRange>&, double), (const));
  MOCK_METHOD(maliput::api::rules::RoadRulebook::QueryResults, DoRules, (), (const));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_METHOD(maliput::api::rules::RightOfWayRule, DoGetRule, (const maliput::api::rules::RightOfWayRule::Id&),
              (const));
  MOCK_METHOD(maliput::api::rules::SpeedLimitRule, DoGetRule, (const maliput::api::rules::SpeedLimitRule::Id&),
              (const));
  MOCK_METHOD(maliput::api::rules::DirectionUsageRule, DoGetRule, (const maliput::api::rules::DirectionUsageRule::Id&),
              (const));
#pragma GCC diagnostic pop
  MOCK_METHOD(maliput::api::rules::DiscreteValueRule, DoGetDiscreteValueRule, (const maliput::api::rules::Rule::Id&),
              (const));
  MOCK_METHOD(maliput::api::rules::RangeValueRule, DoGetRangeValueRule, (const maliput::api::rules::Rule::Id&),
              (const));
};

// @brief Google mock maliput::api::rules::TrafficLightBook.
class TrafficLightBookMock final : public maliput::api::rules::TrafficLightBook {
 public:
  MOCK_METHOD(const maliput::api::rules::TrafficLight*, DoGetTrafficLight,
              (const maliput::api::rules::TrafficLight::Id&), (const));
  MOCK_METHOD(std::vector<const maliput::api::rules::TrafficLight*>, DoTrafficLights, (), (const));
};

// @brief Google mock maliput::api::IntersectionBook.
class IntersectionBookMock final : public maliput::api::IntersectionBook {
 public:
  MOCK_METHOD(std::vector<maliput::api::Intersection*>, DoGetIntersections, ());
  MOCK_METHOD(maliput::api::Intersection*, DoGetIntersection, (const maliput::api::Intersection::Id&));
  MOCK_METHOD(maliput::api::Intersection*, DoGetFindIntersection, (const maliput::api::rules::TrafficLight::Id&));
  MOCK_METHOD(maliput::api::Intersection*, DoGetFindIntersection, (const maliput::api::rules::DiscreteValueRule::Id&));
  MOCK_METHOD(maliput::api::Intersection*, DoGetFindIntersection, (const maliput::api::InertialPosition&));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_METHOD(maliput::api::Intersection*, DoGetFindIntersection, (const maliput::api::rules::RightOfWayRule::Id&));
#pragma GCC diagnostic pop
};

// @brief Google mock maliput::api::rules::PhaseRingBook.
class PhaseRingBookMock : public maliput::api::rules::PhaseRingBook {
 public:
  MOCK_METHOD(std::vector<maliput::api::rules::PhaseRing::Id>, DoGetPhaseRings, (), (const));
  MOCK_METHOD(std::optional<maliput::api::rules::PhaseRing>, DoGetPhaseRing,
              (const maliput::api::rules::PhaseRing::Id&), (const));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_METHOD(std::optional<maliput::api::rules::PhaseRing>, DoFindPhaseRing,
              (const maliput::api::rules::RightOfWayRule::Id&), (const));
#pragma GCC diagnostic pop
  MOCK_METHOD(std::optional<maliput::api::rules::PhaseRing>, DoFindPhaseRing, (const maliput::api::rules::Rule::Id&),
              (const));
};

// @brief Google mock maliput::api::rules::PhaseProvider.
class PhaseProviderMock final : public maliput::api::rules::PhaseProvider {
 public:
  MOCK_METHOD(std::optional<maliput::api::rules::PhaseProvider::Result>, DoGetPhase,
              (const maliput::api::rules::PhaseRing::Id&), (const));
};

// @brief Google mock maliput::api::rules::DiscreteValueRuleStateProvider.
class DiscreteValueRuleStateProviderMock final : public maliput::api::rules::DiscreteValueRuleStateProvider {
 public:
  MOCK_METHOD(std::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult>, DoGetState,
              (const maliput::api::rules::Rule::Id&), (const));
  MOCK_METHOD(std::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult>, DoGetState,
              (const maliput::api::RoadPosition&, const maliput::api::rules::Rule::TypeId&, double), (const));
};

// @brief Google mock maliput::api::rules::RangeValueRuleStateProvider.
class RangeValueRuleStateProviderMock final : public maliput::api::rules::RangeValueRuleStateProvider {
 public:
  MOCK_METHOD(std::optional<maliput::api::rules::RangeValueRuleStateProvider::StateResult>, DoGetState,
              (const maliput::api::rules::Rule::Id&), (const));
  MOCK_METHOD(std::optional<maliput::api::rules::RangeValueRuleStateProvider::StateResult>, DoGetState,
              (const maliput::api::RoadPosition&, const maliput::api::rules::Rule::TypeId&, double), (const));
};

inline std::unique_ptr<api::RoadNetwork> MakeMockedRoadNetwork() {
  return std::make_unique<api::RoadNetwork>(
      std::make_unique<RoadGeometryMock>(), std::make_unique<RoadRulebookMock>(),
      std::make_unique<TrafficLightBookMock>(), std::make_unique<IntersectionBookMock>(),
      std::make_unique<PhaseRingBookMock>(), std::make_unique<PhaseProviderMock>(),
      std::make_unique<maliput::api::rules::RuleRegistry>(), std::make_unique<DiscreteValueRuleStateProviderMock>(),
      std::make_unique<RangeValueRuleStateProviderMock>());
}

}  // namespace test
}  // namespace routing
}  // namespace maliput