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

// TODO: Move this file to a common library for maliput and other dependencies.

#include <memory>
#include <optional>
#include <unordered_map>
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
namespace test {

/// @brief Google mock api::RoadGeometry.
class RoadGeometryMock final : public api::RoadGeometry {
 public:
  MOCK_METHOD(api::RoadGeometryId, do_id, (), (const));
  MOCK_METHOD(int, do_num_junctions, (), (const));
  MOCK_METHOD(const api::Junction*, do_junction, (int), (const));
  MOCK_METHOD(int, do_num_branch_points, (), (const));
  MOCK_METHOD(const api::BranchPoint*, do_branch_point, (int), (const));
  MOCK_METHOD(const api::RoadGeometry::IdIndex&, DoById, (), (const));
  MOCK_METHOD(api::RoadPositionResult, DoToRoadPosition,
              (const api::InertialPosition&, const std::optional<api::RoadPosition>&), (const));
  MOCK_METHOD(std::vector<api::RoadPositionResult>, DoFindRoadPositions, (const api::InertialPosition&, double),
              (const));
  MOCK_METHOD(double, do_linear_tolerance, (), (const));
  MOCK_METHOD(double, do_angular_tolerance, (), (const));
  MOCK_METHOD(double, do_scale_length, (), (const));
  MOCK_METHOD(std::vector<api::InertialPosition>, DoSampleAheadWaypoints, (const api::LaneSRoute&, double), (const));
  MOCK_METHOD(math::Vector3, do_inertial_to_backend_frame_translation, (), (const));
};

/// @brief Google mock api::Lane.
class LaneMock final : public api::Lane {
 public:
  MOCK_METHOD(api::LaneId, do_id, (), (const));
  MOCK_METHOD(int, do_index, (), (const));
  MOCK_METHOD(const api::Segment*, do_segment, (), (const));
  MOCK_METHOD(const api::Lane*, do_to_left, (), (const));
  MOCK_METHOD(const api::Lane*, do_to_right, (), (const));
  MOCK_METHOD(double, do_length, (), (const));
  MOCK_METHOD(const api::BranchPoint*, DoGetBranchPoint, (const api::LaneEnd::Which), (const));
  MOCK_METHOD(std::optional<api::LaneEnd>, DoGetDefaultBranch, (const api::LaneEnd::Which), (const));
  MOCK_METHOD(api::RBounds, do_lane_bounds, (double), (const));
  MOCK_METHOD(api::RBounds, do_segment_bounds, (double), (const));
  MOCK_METHOD(api::HBounds, do_elevation_bounds, (double, double), (const));
  MOCK_METHOD(api::InertialPosition, DoToInertialPosition, (const api::LanePosition&), (const));
  MOCK_METHOD(api::Rotation, DoGetOrientation, (const api::LanePosition&), (const));
  MOCK_METHOD(api::LanePosition, DoEvalMotionDerivatives, (const api::LanePosition&, const api::IsoLaneVelocity&),
              (const));
  MOCK_METHOD(api::LanePositionResult, DoToLanePosition, (const api::InertialPosition&), (const));
  MOCK_METHOD(api::LanePositionResult, DoToSegmentPosition, (const api::InertialPosition&), (const));
  MOCK_METHOD(const api::LaneEndSet*, DoGetConfluentBranches, (const api::LaneEnd::Which), (const));
  MOCK_METHOD(const api::LaneEndSet*, DoGetOngoingBranches, (const api::LaneEnd::Which), (const));
};

class LaneEndSetMock final : public api::LaneEndSet {
 public:
  MOCK_METHOD(int, do_size, (), (const));
  MOCK_METHOD(const api::LaneEnd&, do_get, (int), (const));
};

/// @brief Google mock api::RoadGeometry::IdIndex.
class IdIndexMock final : public api::RoadGeometry::IdIndex {
 public:
  MOCK_METHOD(const api::Lane*, DoGetLane, (const api::LaneId&), (const));
  MOCK_METHOD((const std::unordered_map<api::LaneId, const api::Lane*>&), DoGetLanes, (), (const));
  MOCK_METHOD(const api::Segment*, DoGetSegment, (const api::SegmentId&), (const));
  MOCK_METHOD(const api::Junction*, DoGetJunction, (const api::JunctionId&), (const));
  MOCK_METHOD(const api::BranchPoint*, DoGetBranchPoint, (const api::BranchPointId&), (const));
};

/// @brief Google mock api::Segment.
class SegmentMock : public api::Segment {
 public:
  MOCK_METHOD(api::SegmentId, do_id, (), (const));
  MOCK_METHOD(const api::Junction*, do_junction, (), (const));
  MOCK_METHOD(int, do_num_lanes, (), (const));
  MOCK_METHOD(const api::Lane*, do_lane, (int), (const));
};

// @brief Google mock api::rules::RoadRulebook.
class RoadRulebookMock final : public api::rules::RoadRulebook {
 public:
  MOCK_METHOD(api::rules::RoadRulebook::QueryResults, DoFindRules, (const std::vector<api::LaneSRange>&, double),
              (const));
  MOCK_METHOD(api::rules::RoadRulebook::QueryResults, DoRules, (), (const));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_METHOD(api::rules::RightOfWayRule, DoGetRule, (const api::rules::RightOfWayRule::Id&), (const));
  MOCK_METHOD(api::rules::SpeedLimitRule, DoGetRule, (const api::rules::SpeedLimitRule::Id&), (const));
  MOCK_METHOD(api::rules::DirectionUsageRule, DoGetRule, (const api::rules::DirectionUsageRule::Id&), (const));
#pragma GCC diagnostic pop
  MOCK_METHOD(api::rules::DiscreteValueRule, DoGetDiscreteValueRule, (const api::rules::Rule::Id&), (const));
  MOCK_METHOD(api::rules::RangeValueRule, DoGetRangeValueRule, (const api::rules::Rule::Id&), (const));
};

// @brief Google mock api::rules::TrafficLightBook.
class TrafficLightBookMock final : public api::rules::TrafficLightBook {
 public:
  MOCK_METHOD(const api::rules::TrafficLight*, DoGetTrafficLight, (const api::rules::TrafficLight::Id&), (const));
  MOCK_METHOD(std::vector<const api::rules::TrafficLight*>, DoTrafficLights, (), (const));
};

// @brief Google mock api::IntersectionBook.
class IntersectionBookMock final : public api::IntersectionBook {
 public:
  MOCK_METHOD(std::vector<api::Intersection*>, DoGetIntersections, ());
  MOCK_METHOD(api::Intersection*, DoGetIntersection, (const api::Intersection::Id&));
  MOCK_METHOD(api::Intersection*, DoGetFindIntersection, (const api::rules::TrafficLight::Id&));
  MOCK_METHOD(api::Intersection*, DoGetFindIntersection, (const api::rules::DiscreteValueRule::Id&));
  MOCK_METHOD(api::Intersection*, DoGetFindIntersection, (const api::InertialPosition&));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_METHOD(api::Intersection*, DoGetFindIntersection, (const api::rules::RightOfWayRule::Id&));
#pragma GCC diagnostic pop
};

// @brief Google mock api::rules::PhaseRingBook.
class PhaseRingBookMock : public api::rules::PhaseRingBook {
 public:
  MOCK_METHOD(std::vector<api::rules::PhaseRing::Id>, DoGetPhaseRings, (), (const));
  MOCK_METHOD(std::optional<api::rules::PhaseRing>, DoGetPhaseRing, (const api::rules::PhaseRing::Id&), (const));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_METHOD(std::optional<api::rules::PhaseRing>, DoFindPhaseRing, (const api::rules::RightOfWayRule::Id&), (const));
#pragma GCC diagnostic pop
  MOCK_METHOD(std::optional<api::rules::PhaseRing>, DoFindPhaseRing, (const api::rules::Rule::Id&), (const));
};

// @brief Google mock api::rules::PhaseProvider.
class PhaseProviderMock final : public api::rules::PhaseProvider {
 public:
  MOCK_METHOD(std::optional<api::rules::PhaseProvider::Result>, DoGetPhase, (const api::rules::PhaseRing::Id&),
              (const));
};

// @brief Google mock api::rules::DiscreteValueRuleStateProvider.
class DiscreteValueRuleStateProviderMock final : public api::rules::DiscreteValueRuleStateProvider {
 public:
  MOCK_METHOD(std::optional<api::rules::DiscreteValueRuleStateProvider::StateResult>, DoGetState,
              (const api::rules::Rule::Id&), (const));
  MOCK_METHOD(std::optional<api::rules::DiscreteValueRuleStateProvider::StateResult>, DoGetState,
              (const api::RoadPosition&, const api::rules::Rule::TypeId&, double), (const));
};

// @brief Google mock api::rules::RangeValueRuleStateProvider.
class RangeValueRuleStateProviderMock final : public api::rules::RangeValueRuleStateProvider {
 public:
  MOCK_METHOD(std::optional<api::rules::RangeValueRuleStateProvider::StateResult>, DoGetState,
              (const api::rules::Rule::Id&), (const));
  MOCK_METHOD(std::optional<api::rules::RangeValueRuleStateProvider::StateResult>, DoGetState,
              (const api::RoadPosition&, const api::rules::Rule::TypeId&, double), (const));
};

inline std::unique_ptr<api::RoadNetwork> MakeMockedRoadNetwork() {
  return std::make_unique<api::RoadNetwork>(
      std::make_unique<RoadGeometryMock>(), std::make_unique<RoadRulebookMock>(),
      std::make_unique<TrafficLightBookMock>(), std::make_unique<IntersectionBookMock>(),
      std::make_unique<PhaseRingBookMock>(), std::make_unique<PhaseProviderMock>(),
      std::make_unique<api::rules::RuleRegistry>(), std::make_unique<DiscreteValueRuleStateProviderMock>(),
      std::make_unique<RangeValueRuleStateProviderMock>());
}

}  // namespace test
}  // namespace maliput
