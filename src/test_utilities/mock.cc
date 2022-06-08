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
#include "maliput/test_utilities/mock.h"

#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "maliput/api/branch_point.h"
#include "maliput/api/intersection.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace test {
namespace {

using rules::Bulb;
using rules::BulbGroup;
using rules::DirectionUsageRule;
using rules::DiscreteValueRule;
using rules::Phase;
using rules::PhaseRing;
using rules::RangeValueRule;
using rules::RightOfWayRule;
using rules::Rule;
using rules::SpeedLimitRule;
using rules::TrafficLight;

class MockOneLaneIdIndex final : public RoadGeometry::IdIndex {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockOneLaneIdIndex);
  MockOneLaneIdIndex() : RoadGeometry::IdIndex() {}

 private:
  const Lane* DoGetLane(const LaneId&) const override { return &mock_lane_; }

  const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override { return lane_map_; }

  const Segment* DoGetSegment(const SegmentId&) const override { return nullptr; };
  const Junction* DoGetJunction(const JunctionId&) const override { return nullptr; };
  const BranchPoint* DoGetBranchPoint(const BranchPointId&) const override { return nullptr; }

  const MockLane mock_lane_{LaneId{"mock"}};
  const std::unordered_map<LaneId, const Lane*> lane_map_{{LaneId("mock"), &mock_lane_}};
};

class MockOneLaneRoadGeometry final : public RoadGeometry {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockOneLaneRoadGeometry)
  MockOneLaneRoadGeometry() {}

 private:
  RoadGeometryId do_id() const override { return RoadGeometryId("mock"); }
  int do_num_junctions() const override { return 1; }
  const Junction* do_junction(int) const override { return nullptr; };
  int do_num_branch_points() const override { return 1; }
  const BranchPoint* do_branch_point(int) const override { return nullptr; }
  const IdIndex& DoById() const override { return mock_id_index_; }
  api::RoadPositionResult DoToRoadPosition(const InertialPosition&, const std::optional<RoadPosition>&) const override {
    return api::RoadPositionResult();
  }
  std::vector<api::RoadPositionResult> DoFindRoadPositions(const InertialPosition&, double) const override {
    return {api::RoadPositionResult()};
  }
  double do_linear_tolerance() const override { return 0; }
  double do_angular_tolerance() const override { return 0; }
  double do_scale_length() const override { return 0; }
  math::Vector3 do_inertial_to_backend_frame_translation() const override { return math::Vector3{0., 0., 0.}; }

  MockOneLaneIdIndex mock_id_index_;
};

class MockRoadRulebook : public rules::RoadRulebook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadRulebook)
  MockRoadRulebook() {}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  void set_right_of_way(const RightOfWayRule& rule) { right_of_way_rule_ = rule; }
  void set_direction_usage(const DirectionUsageRule& rule) { direction_usage_rule_ = rule; }
  void set_speed_limit(const SpeedLimitRule& rule) { speed_limit_rule_ = rule; }
#pragma GCC diagnostic pop
  void set_discrete_value_rule(const DiscreteValueRule& rule) { discrete_value_rule_ = rule; }
  void set_range_value_rule(const RangeValueRule& rule) { range_value_rule_ = rule; }

 private:
  QueryResults DoFindRules(const std::vector<LaneSRange>&, double) const override { return DoRules(); }
  QueryResults DoRules() const override {
    QueryResults result;
    if (right_of_way_rule_.has_value()) {
      result.right_of_way.emplace(right_of_way_rule_->id(), *right_of_way_rule_);
    }
    if (direction_usage_rule_.has_value()) {
      result.direction_usage.emplace(direction_usage_rule_->id(), *direction_usage_rule_);
    }
    if (speed_limit_rule_.has_value()) {
      result.speed_limit.emplace(speed_limit_rule_->id(), *speed_limit_rule_);
    }
    if (discrete_value_rule_.has_value()) {
      result.discrete_value_rules.emplace(discrete_value_rule_->id(), *discrete_value_rule_);
    }
    if (range_value_rule_.has_value()) {
      result.range_value_rules.emplace(range_value_rule_->id(), *range_value_rule_);
    }
    return result;
  }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  RightOfWayRule DoGetRule(const RightOfWayRule::Id&) const override { return *right_of_way_rule_; }
  SpeedLimitRule DoGetRule(const SpeedLimitRule::Id&) const override { return *speed_limit_rule_; }
  DirectionUsageRule DoGetRule(const DirectionUsageRule::Id&) const override { return *direction_usage_rule_; }
#pragma GCC diagnostic pop
  DiscreteValueRule DoGetDiscreteValueRule(const Rule::Id& id) const override {
    if (discrete_value_rule_.has_value() && discrete_value_rule_->id() == id) {
      return *discrete_value_rule_;
    }
    throw std::out_of_range("Unknown Id.");
  }
  RangeValueRule DoGetRangeValueRule(const Rule::Id& id) const override {
    if (range_value_rule_.has_value() && range_value_rule_->id() == id) {
      return *range_value_rule_;
    }
    throw std::out_of_range("Unknown Id.");
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::optional<RightOfWayRule> right_of_way_rule_{};
  std::optional<DirectionUsageRule> direction_usage_rule_{};
  std::optional<SpeedLimitRule> speed_limit_rule_{};
#pragma GCC diagnostic pop
  std::optional<DiscreteValueRule> discrete_value_rule_{};
  std::optional<RangeValueRule> range_value_rule_{};
};

// Creates rules
// based on CreateMockContiguousRoadGeometry() info.
class MockContiguityRoadRulebook final : public rules::RoadRulebook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockContiguityRoadRulebook)
  MockContiguityRoadRulebook() {}

  void set_discrete_value_rule(const DiscreteValueRule& rule) { discrete_value_rule_ = rule; }
  void set_range_value_rule(const RangeValueRule& rule) { range_value_rule_ = rule; }

 private:
  QueryResults DoFindRules(const std::vector<LaneSRange>&, double) const override { return DoRules(); }
  QueryResults DoRules() const override {
    QueryResults result;
    if (discrete_value_rule_.has_value()) {
      result.discrete_value_rules.emplace(discrete_value_rule_->id(), *discrete_value_rule_);
    }
    if (range_value_rule_.has_value()) {
      result.range_value_rules.emplace(range_value_rule_->id(), *range_value_rule_);
    }
    return result;
  }

  DiscreteValueRule DoGetDiscreteValueRule(const Rule::Id& id) const override {
    if (discrete_value_rule_.has_value() && discrete_value_rule_->id() == id) {
      return *discrete_value_rule_;
    }
    throw std::out_of_range("Unknown Id.");
  }
  RangeValueRule DoGetRangeValueRule(const Rule::Id& id) const override {
    if (range_value_rule_.has_value() && range_value_rule_->id() == id) {
      return *range_value_rule_;
    }
    throw std::out_of_range("Unknown Id.");
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  RightOfWayRule DoGetRule(const RightOfWayRule::Id& id) const override { throw std::out_of_range("Unknown Id."); }
  SpeedLimitRule DoGetRule(const SpeedLimitRule::Id& id) const override { throw std::out_of_range("Unknown Id."); }
  DirectionUsageRule DoGetRule(const DirectionUsageRule::Id& id) const override {
    throw std::out_of_range("Unknown Id.");
  }
#pragma GCC diagnostic pop

  std::optional<DiscreteValueRule> discrete_value_rule_{};
  std::optional<RangeValueRule> range_value_rule_{};
};

class MockTrafficLightBook final : public rules::TrafficLightBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockTrafficLightBook)
  MockTrafficLightBook() = default;
  void set_traffic_light(std::unique_ptr<TrafficLight> traffic_light) { traffic_light_ = std::move(traffic_light); }

 private:
  const TrafficLight* DoGetTrafficLight(const TrafficLight::Id& id) const override {
    return (traffic_light_.get() != nullptr && traffic_light_->id() == id) ? traffic_light_.get() : nullptr;
  }
  std::vector<const TrafficLight*> DoTrafficLights() const override { return {traffic_light_.get()}; }

  std::unique_ptr<TrafficLight> traffic_light_{};
};

class MockPhaseRingBook final : public rules::PhaseRingBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockPhaseRingBook)
  MockPhaseRingBook() {}

  void SetPhaseRing(const rules::PhaseRing& phase_ring) { phase_rings_.emplace(phase_ring.id(), phase_ring); }

 private:
  std::vector<PhaseRing::Id> DoGetPhaseRings() const override {
    std::vector<PhaseRing::Id> phase_ring_ids;
    for (const auto& k_v : phase_rings_) {
      phase_ring_ids.push_back(k_v.first);
    }
    return phase_ring_ids;
  }

  std::optional<rules::PhaseRing> DoGetPhaseRing(const rules::PhaseRing::Id& id) const override {
    return phase_rings_.find(id) != phase_rings_.end() ? std::optional<rules::PhaseRing>{phase_rings_.at(id)}
                                                       : std::optional<rules::PhaseRing>{};
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::optional<rules::PhaseRing> DoFindPhaseRing(const rules::RightOfWayRule::Id&) const override {
    return std::nullopt;
  }
#pragma GCC diagnostic pop

  std::optional<rules::PhaseRing> DoFindPhaseRing(const rules::Rule::Id&) const override { return std::nullopt; }

  std::unordered_map<rules::PhaseRing::Id, rules::PhaseRing> phase_rings_;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
class MockRightOfWayRuleStateProvider final : public rules::RightOfWayRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockRightOfWayRuleStateProvider)
  MockRightOfWayRuleStateProvider() {}

 private:
  std::optional<RightOfWayResult> DoGetState(const RightOfWayRule::Id&) const override { return std::nullopt; }
};
#pragma GCC diagnostic pop

class MockPhaseProvider final : public rules::PhaseProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockPhaseProvider)
  MockPhaseProvider() {}

 private:
  std::optional<Result> DoGetPhase(const rules::PhaseRing::Id&) const override { return std::nullopt; }
};

class MockIntersection final : public Intersection {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockIntersection)
  MockIntersection(const Intersection::Id& id, const rules::PhaseRing& ring) : Intersection(id, {}, ring) {}

 private:
  std::optional<rules::PhaseProvider::Result> Phase() const override { return std::nullopt; }

  void SetPhase(const api::rules::Phase::Id&, const std::optional<api::rules::Phase::Id>& next_phase = std::nullopt,
                const std::optional<double>& duration_until = std::nullopt) override {}
};

class MockIntersectionBook final : public IntersectionBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockIntersectionBook);
  MockIntersectionBook() : intersection_(Intersection::Id("Mock"), CreatePhaseRing()) {}

 private:
  std::vector<api::Intersection*> DoGetIntersections() override {
    std::vector<api::Intersection*> result(1, &intersection_);
    return result;
  }

  api::Intersection* DoGetIntersection(const api::Intersection::Id& id) override {
    if (id == intersection_.id()) {
      return &intersection_;
    }
    return nullptr;
  }
  api::Intersection* DoGetFindIntersection(const api::rules::TrafficLight::Id& id) override { return nullptr; };

  api::Intersection* DoGetFindIntersection(const api::rules::DiscreteValueRule::Id& id) override { return nullptr; };

  api::Intersection* DoGetFindIntersection(const api::InertialPosition& inertial_pose) override { return nullptr; };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  api::Intersection* DoGetFindIntersection(const api::rules::RightOfWayRule::Id& id) override { return nullptr; };
#pragma GCC diagnostic pop

  MockIntersection intersection_;
};

class MockDiscreteValueRuleStateProvider : public rules::DiscreteValueRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockDiscreteValueRuleStateProvider);
  MockDiscreteValueRuleStateProvider() = default;

 private:
  std::optional<rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(const Rule::Id&) const override {
    return std::nullopt;
  }
  std::optional<rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(
      const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type,
      double tolerance) const override {
    return std::nullopt;
  }
};

class MockRangeValueRuleStateProvider : public rules::RangeValueRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockRangeValueRuleStateProvider);
  MockRangeValueRuleStateProvider() = default;

 private:
  std::optional<rules::RangeValueRuleStateProvider::StateResult> DoGetState(const Rule::Id&) const override {
    return std::nullopt;
  }
  std::optional<rules::RangeValueRuleStateProvider::StateResult> DoGetState(const api::RoadPosition& road_position,
                                                                            const api::rules::Rule::TypeId& rule_type,
                                                                            double tolerance) const override {
    return std::nullopt;
  }
};

}  // namespace

LaneSRoute CreateLaneSRoute() {
  return LaneSRoute({LaneSRange(LaneId("a"), {0., 9.}), LaneSRange(LaneId("b"), {17., 12.})});
}

LaneSRange CreateLaneSRange() { return LaneSRange(LaneId("a"), {0., 9.}); }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
RightOfWayRule::State::YieldGroup YieldGroup2() {
  return {RightOfWayRule::Id("other_rule_a"), RightOfWayRule::Id("other_rule_b")};
}

RightOfWayRule::State NoYieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s1"), RightOfWayRule::State::Type::kStop, {});
}

RightOfWayRule::State YieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s2"), RightOfWayRule::State::Type::kGo, YieldGroup2());
}

RightOfWayRule::RelatedBulbGroups RelatedBulbGroups() {
  return rules::RightOfWayRule::RelatedBulbGroups{
      {rules::TrafficLight::Id("TrafficLightId"), {rules::BulbGroup::Id("BulbGroupId")}}};
}

RightOfWayRule CreateRightOfWayRule() { return CreateRightOfWayRule(RightOfWayBuildFlags{}); }

RightOfWayRule CreateRightOfWayRule(const RightOfWayBuildFlags& build_flags) {
  return RightOfWayRule(
      RightOfWayRule::Id("mock_id"), CreateLaneSRoute(), RightOfWayRule::ZoneType::kStopExcluded,
      {NoYieldState(), YieldState()},
      build_flags.add_related_bulb_groups ? RelatedBulbGroups() : RightOfWayRule::RelatedBulbGroups{});
}

SpeedLimitRule CreateSpeedLimitRule() {
  return SpeedLimitRule(SpeedLimitRule::Id("some_id"), CreateLaneSRange(), SpeedLimitRule::Severity::kStrict, 33., 77.);
}

DirectionUsageRule::State CreateDirectionUsageRuleState() {
  return DirectionUsageRule::State(DirectionUsageRule::State::Id("dur_state"), DirectionUsageRule::State::Type::kWithS,
                                   DirectionUsageRule::State::Severity::kStrict);
}

DirectionUsageRule CreateDirectionUsageRule() {
  return DirectionUsageRule(DirectionUsageRule::Id("dur_id"), CreateLaneSRange(), {CreateDirectionUsageRuleState()});
}
#pragma GCC diagnostic pop

rules::Rule::RelatedRules CreateEmptyRelatedRules() { return {}; }

rules::Rule::RelatedRules CreateNonEmptyRelatedRules() {
  return Rule::RelatedRules{{"RelatedRulesGroup", {Rule::Id("RuleTypeIdA/RuleIdA"), Rule::Id("RuleTypeIdB/RuleIdB")}}};
}

rules::Rule::RelatedUniqueIds CreateEmptyRelatedUniqueIds() { return {}; }

rules::Rule::RelatedUniqueIds CreateNonEmptyRelatedUniqueIds() {
  return Rule::RelatedUniqueIds{
      {"UniqueBulbGroupId",
       {rules::UniqueBulbGroupId(TrafficLight::Id("TrafficLightIdA"), BulbGroup::Id("BulbGroupIdA"))}}};
}

DiscreteValueRule CreateDiscreteValueRule() {
  return DiscreteValueRule(Rule::Id("dvrt/dvr_id"), Rule::TypeId("dvrt"), CreateLaneSRoute(),
                           {DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                                             CreateEmptyRelatedUniqueIds(), "value1"},
                            DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                                             CreateEmptyRelatedUniqueIds(), "value2"}});
}

DiscreteValueRule CreateDiscreteValueRule(bool consistent_related_rule_groups) {
  if (consistent_related_rule_groups) {
    return DiscreteValueRule(
        Rule::Id("dvrt/dvr_id"), Rule::TypeId("dvrt"), CreateLaneSRoute(),
        {DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict,
                                          Rule::RelatedRules{{"RelatedRulesGroup", {Rule::Id("dvrt/dvr_id")}}},
                                          CreateEmptyRelatedUniqueIds(), "value1"},
         DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                          CreateEmptyRelatedUniqueIds(), "value2"}});
  }
  return DiscreteValueRule(
      Rule::Id("dvrt/dvr_id"), Rule::TypeId("dvrt"), CreateLaneSRoute(),
      {DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict,
                                        Rule::RelatedRules{{"RelatedRulesGroup", {Rule::Id("dvrt/DoesNotExist")}}},
                                        CreateEmptyRelatedUniqueIds(), "value1"},
       DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                        CreateEmptyRelatedUniqueIds(), "value2"}});
}

DiscreteValueRule CreateDiscreteValueRuleForContiguityTest() {
  return DiscreteValueRule(
      Rule::Id("dvrt/dvr_id"), Rule::TypeId("dvrt"),
      LaneSRoute({LaneSRange(LaneId("mock_a"), {0., 10.}), LaneSRange(LaneId("mock_b"), {0., 10.})}),
      {DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                        CreateEmptyRelatedUniqueIds(), "value1"},
       DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                        CreateEmptyRelatedUniqueIds(), "value2"}});
}

RangeValueRule::Range CreateRange() {
  return RangeValueRule::Range{rules::Rule::State::kStrict,
                               CreateEmptyRelatedRules(),
                               CreateEmptyRelatedUniqueIds(),
                               "description",
                               123. /* min */,
                               456. /* max */};
}

RangeValueRule CreateRangeValueRule() {
  return RangeValueRule(Rule::Id("rvrt/rvr_id"), Rule::TypeId("rvrt"), CreateLaneSRoute(), {CreateRange()});
}

RangeValueRule CreateRangeValueRule(bool consistent_related_rule_groups) {
  if (consistent_related_rule_groups) {
    return RangeValueRule(
        Rule::Id("rvrt/rvr_id"), Rule::TypeId("rvrt"), CreateLaneSRoute(),
        {RangeValueRule::Range{rules::Rule::State::kStrict,
                               Rule::RelatedRules{{"RelatedRulesGroup", {Rule::Id("rvrt/rvr_id")}}},
                               CreateEmptyRelatedUniqueIds(), "description", 123. /* min */, 456. /* max */}});
  }
  return RangeValueRule(
      Rule::Id("rvrt/rvr_id"), Rule::TypeId("rvrt"), CreateLaneSRoute(),
      {RangeValueRule::Range{rules::Rule::State::kStrict,
                             Rule::RelatedRules{{"RelatedRulesGroup", {Rule::Id("rvrt/DoesNotExist")}}},
                             CreateEmptyRelatedUniqueIds(), "description", 123. /* min */, 456. /* max */}});
}

RangeValueRule CreateRangeValueRuleForContiguityTest() {
  return RangeValueRule(Rule::Id("rvrt/rvr_id"), Rule::TypeId("rvrt"),
                        LaneSRoute({LaneSRange(LaneId("mock_a"), {0., 10.}), LaneSRange(LaneId("mock_b"), {0., 10.})}),
                        {CreateRange()});
}

Phase CreatePhase() { return Phase(Phase::Id("mock"), {}, {}); }

Phase CreatePhase(const PhaseBuildFlags& build_flags) {
  const Rule::Id rule_id(build_flags.add_missing_rule ? "dvrt/unknown_id" : "dvrt/dvr_id");
  const auto discrete_value =
      build_flags.add_missing_value
          ? DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                             CreateEmptyRelatedUniqueIds(), "unkonwn"}
          : DiscreteValueRule::DiscreteValue{rules::Rule::State::kStrict, CreateEmptyRelatedRules(),
                                             CreateEmptyRelatedUniqueIds(), "value1"};
  const rules::UniqueBulbId bulb_id(
      rules::TrafficLight::Id("TrafficLightId"), rules::BulbGroup::Id("BulbGroupId"),
      build_flags.add_missing_bulb ? rules::Bulb::Id("UnknownBulbId") : rules::Bulb::Id("BulbId"));
  const rules::UniqueBulbId unique_bulb_id(
      rules::TrafficLight::Id("TrafficLightId"), rules::BulbGroup::Id("BulbGroupId"),
      build_flags.add_missing_bulb ? rules::Bulb::Id("UnknownBulbId") : rules::Bulb::Id("BulbId"));
  const rules::BulbState bulb_state =
      build_flags.add_missing_bulb_state ? rules::BulbState::kOff : rules::BulbState::kOn;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  return Phase(Phase::Id("phase_id"), rules::RuleStates{}, rules::DiscreteValueRuleStates{{rule_id, {discrete_value}}},
               rules::BulbStates{{unique_bulb_id, bulb_state}});
#pragma GCC diagnostic pop
}

PhaseRing CreatePhaseRing() { return PhaseRing(PhaseRing::Id("mock"), {CreatePhase()}); }

PhaseRing CreatePhaseRing(const PhaseBuildFlags& build_flags) {
  return PhaseRing(PhaseRing::Id("mock"), {CreatePhase(build_flags)});
}

std::unique_ptr<Lane> CreateLane(const LaneId& id) { return std::make_unique<MockLane>(id); }

std::unique_ptr<RoadGeometry> CreateMultipleLanesRoadGeometry(const std::vector<Lane*>& lanes) {
  auto rg = std::make_unique<MockRoadGeometry>(RoadGeometryId("mock"));
  for (const auto& lane : lanes) {
    rg->GetIdIndex()->add_lane_to_map(lane->id(), lane);
  }
  return std::move(rg);
}

std::unique_ptr<RoadNetwork> CreateRoadNetwork() {
  return std::make_unique<RoadNetwork>(
      std::move(CreateRoadGeometry()), std::move(CreateRoadRulebook()), std::move(CreateTrafficLightBook()),
      std::move(CreateIntersectionBook()), std::move(CreatePhaseRingBook()),
      std::move(CreateRightOfWayRuleStateProvider()), std::move(CreatePhaseProvider()), std::move(CreateRuleRegistry()),
      std::move(CreateDiscreteValueRuleStateProvider()), std::move(CreateRangeValueRuleStateProvider()));
}

std::unique_ptr<RoadGeometry> CreateRoadGeometry() {
  return std::make_unique<MockRoadGeometry>(RoadGeometryId("mock"));
}

std::unique_ptr<RoadGeometry> CreateRoadGeometry(const RoadGeometryBuildFlags& build_flags) {
  auto rg = std::make_unique<MockRoadGeometry>(RoadGeometryId("mock"));
  auto* id_index = rg->GetIdIndex();

  if (build_flags.add_branchpoint) {
    auto start_bp = std::make_unique<MockBranchPoint>(BranchPointId("mock_start"));
    start_bp->set_road_geometry(rg.get());
    if (build_flags.id_index_build_flags.add_branchpoint) {
      id_index->add_branchpoint_to_map(start_bp->id(), start_bp.get());
    }
    rg->set_start_bp(std::move(start_bp));
    auto end_bp = std::make_unique<MockBranchPoint>(BranchPointId("mock_end"));
    end_bp->set_road_geometry(rg.get());
    if (build_flags.id_index_build_flags.add_branchpoint) {
      id_index->add_branchpoint_to_map(end_bp->id(), end_bp.get());
    }
    rg->set_end_bp(std::move(end_bp));
  }
  if (build_flags.add_junction) {
    auto junction = std::make_unique<MockJunction>(JunctionId("mock"));
    junction->set_road_geometry(rg.get());
    if (build_flags.id_index_build_flags.add_junction) {
      id_index->add_junction_to_map(junction->id(), junction.get());
    }
    if (build_flags.add_segment) {
      auto segment = std::make_unique<MockSegment>(SegmentId("mock"));
      segment->set_junction(junction.get());
      if (build_flags.id_index_build_flags.add_segment) {
        id_index->add_segment_to_map(segment->id(), segment.get());
      }
      if (build_flags.add_lane) {
        auto lane = std::make_unique<MockLane>(LaneId("mock"));
        lane->set_segment(segment.get());
        if (build_flags.id_index_build_flags.add_lane) {
          id_index->add_lane_to_map(lane->id(), lane.get());
        }
        if (build_flags.add_branchpoint && build_flags.add_lane_end_set) {
          auto lane_end_set_start = std::make_unique<MockLaneEndSet>();
          lane_end_set_start->set_lane_end(LaneEnd(lane.get(), LaneEnd::Which::kStart));
          rg->start_bp()->set_lane_end_set_a(std::move(lane_end_set_start));
          rg->start_bp()->set_lane_end_set_b(std::make_unique<MockLaneEndSet>());
          auto lane_end_set_end = std::make_unique<MockLaneEndSet>();
          lane_end_set_end->set_lane_end(LaneEnd(lane.get(), LaneEnd::Which::kFinish));
          rg->end_bp()->set_lane_end_set_a(std::move(lane_end_set_end));
          rg->end_bp()->set_lane_end_set_b(std::make_unique<MockLaneEndSet>());
          lane->set_start_bp(rg->start_bp());
          lane->set_end_bp(rg->end_bp());
        }
        segment->set_lane(std::move(lane));
      }
      junction->set_segment(std::move(segment));
    }
    rg->add_junction(std::move(junction));
  }
  return std::move(rg);
}

// @{ Creates a RoadGeometry with two Junctions. Each Junction will have a Segment and a Lane.
//    Return values when ToLanePosition method is called are hardcoded or passed via arguments.
std::unique_ptr<RoadGeometry> CreateTwoLanesRoadGeometry() {
  const LanePositionResult lane_a_pos_result{{10., 20., 30.}, {12., 89., 1.}, 0.5};
  const LanePositionResult lane_b_pos_result{{40., 50., 60.}, {50., 1., 45.}, 30.};
  return CreateTwoLanesRoadGeometry(lane_a_pos_result, lane_b_pos_result);
}

std::unique_ptr<RoadGeometry> CreateTwoLanesRoadGeometry(const LanePositionResult& lane_a_pos_result,
                                                         const LanePositionResult& lane_b_pos_result) {
  constexpr double kArbitrary{1.};
  const math::Vector3 kZero{0., 0., 0.};
  auto rg = std::make_unique<MockRoadGeometry>(RoadGeometryId("road_geometry"), kArbitrary, kArbitrary, kZero);
  auto junction_a = std::make_unique<MockJunction>(JunctionId("junction_a"));
  auto junction_b = std::make_unique<MockJunction>(JunctionId("junction_b"));
  auto segment_a = std::make_unique<MockSegment>(SegmentId("segment_a"));
  auto segment_b = std::make_unique<MockSegment>(SegmentId("segment_b"));
  auto lane_a = std::make_unique<MockLane>(LaneId("lane_a"), lane_a_pos_result);
  auto lane_b = std::make_unique<MockLane>(LaneId("lane_b"), lane_b_pos_result);
  junction_a->set_road_geometry(rg.get());
  junction_b->set_road_geometry(rg.get());
  segment_a->set_junction(junction_a.get());
  segment_b->set_junction(junction_b.get());
  lane_a->set_segment(segment_a.get());
  lane_b->set_segment(segment_b.get());
  rg->GetIdIndex()->add_lane_to_map(lane_a->id(), lane_a.get());
  rg->GetIdIndex()->add_lane_to_map(lane_b->id(), lane_b.get());
  segment_a->set_lane(std::move(lane_a));
  segment_b->set_lane(std::move(lane_b));
  junction_a->set_segment(std::move(segment_a));
  junction_b->set_segment(std::move(segment_b));
  rg->add_junction(std::move(junction_a));
  rg->add_junction(std::move(junction_b));
  return std::move(rg);
}
// @}

// Creates a RoadGeometry with two Junctions. Each Junction will have a Segment and a Lane.
// Lanes' geometry will be created in accordance to `build_flags`.
std::unique_ptr<RoadGeometry> CreateMockContiguousRoadGeometry(const RoadGeometryContiguityBuildFlags& build_flags) {
  // Creates the road geometry.
  auto rg = std::make_unique<MockRoadGeometry>(RoadGeometryId("mock"), build_flags.linear_tolerance,
                                               build_flags.angular_tolerance,
                                               build_flags.inertial_to_backend_frame_translation);
  // Creates the Junctions.
  auto junction_a = std::make_unique<MockJunction>(JunctionId("mock_a"));
  junction_a->set_road_geometry(rg.get());
  auto junction_b = std::make_unique<MockJunction>(JunctionId("mock_b"));
  junction_b->set_road_geometry(rg.get());
  // Creates the Segments.
  auto segment_a = std::make_unique<MockSegment>(SegmentId("mock_a"));
  segment_a->set_junction(junction_a.get());
  auto segment_b = std::make_unique<MockSegment>(SegmentId("mock_b"));
  segment_b->set_junction(junction_b.get());
  // Creates the start and end points of the Lanes.
  InertialPosition start_ip_a;
  InertialPosition end_ip_a;
  InertialPosition start_ip_b;
  InertialPosition end_ip_b;
  Rotation start_rot_a;
  Rotation end_rot_a;
  Rotation start_rot_b;
  Rotation end_rot_b;

  if (build_flags.add_linear_mismatch) {
    // Assign different positions for the lane 1 end and lane 2 start.
    start_ip_a = InertialPosition(0, 0, 0);
    end_ip_a = InertialPosition(10, 0, 0);
    start_ip_b = InertialPosition(11, 0, 0);
    end_ip_b = InertialPosition(21, 0, 0);
  } else {
    // Assign same position for the lane 1 end and lane 2 start.
    start_ip_a = InertialPosition(0, 0, 0);
    end_ip_a = InertialPosition(10, 0, 0);
    start_ip_b = InertialPosition(10, 0, 0);
    end_ip_b = InertialPosition(20, 0, 0);
  }
  if (build_flags.add_angular_mismatch) {
    // Assign different rotations for the lane 1 end and lane 2 start.
    start_rot_a = Rotation(Rotation::FromRpy(0.0, 0.0, 0.0));
    end_rot_a = Rotation(Rotation::FromRpy(0.0, 0.0, 0.0));
    start_rot_b = Rotation(Rotation::FromRpy(0.0, 0.0, M_PI / 2.0));
    end_rot_b = Rotation(Rotation::FromRpy(0.0, 0.0, M_PI / 2.0));
  } else {
    // Assign same rotation for the lane 1 end and lane 2 start.
    start_rot_a = Rotation(Rotation::FromRpy(0.0, 0.0, 0.0));
    end_rot_a = Rotation(Rotation::FromRpy(0.0, 0.0, 0.0));
    start_rot_b = Rotation(Rotation::FromRpy(0.0, 0.0, 0.0));
    end_rot_b = Rotation(Rotation::FromRpy(0.0, 0.0, 0.0));
  }
  auto lane_a = std::make_unique<MockLane>(LaneId("mock_a"), start_ip_a, start_rot_a, end_ip_a, end_rot_a);
  auto lane_b = std::make_unique<MockLane>(LaneId("mock_b"), start_ip_b, start_rot_b, end_ip_b, end_rot_b);
  lane_a->set_segment(segment_a.get());
  lane_b->set_segment(segment_b.get());
  rg->GetIdIndex()->add_lane_to_map(lane_a->id(), lane_a.get());
  rg->GetIdIndex()->add_lane_to_map(lane_b->id(), lane_b.get());
  segment_a->set_lane(std::move(lane_a));
  segment_b->set_lane(std::move(lane_b));
  junction_b->set_segment(std::move(segment_b));
  junction_a->set_segment(std::move(segment_a));
  rg->add_junction(std::move(junction_a));
  rg->add_junction(std::move(junction_b));
  return std::move(rg);
}
// Creates a Road Rulebook for contiguity test according to the building flags.
std::unique_ptr<rules::RoadRulebook> CreateMockContiguousRoadRulebook(
    const RoadRulebookContiguityBuildFlags& build_flags) {
  auto rulebook = std::make_unique<MockContiguityRoadRulebook>();
  if (build_flags.add_discrete_value_rule) {
    rulebook->set_discrete_value_rule(CreateDiscreteValueRuleForContiguityTest());
  }
  if (build_flags.add_range_value_rule) {
    rulebook->set_range_value_rule(CreateRangeValueRuleForContiguityTest());
  }
  return std::move(rulebook);
}

std::unique_ptr<RoadGeometry> CreateOneLaneRoadGeometry() { return std::make_unique<MockOneLaneRoadGeometry>(); }

std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook() { return CreateRoadRulebook(RoadRulebookBuildFlags{}); }

std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook(const RoadRulebookBuildFlags& build_flags) {
  auto rulebook = std::make_unique<MockRoadRulebook>();
  if (build_flags.add_right_of_way) {
    rulebook->set_right_of_way(CreateRightOfWayRule(build_flags.right_of_way_build_flags));
  }
  if (build_flags.add_direction_usage) {
    rulebook->set_direction_usage(CreateDirectionUsageRule());
  }
  if (build_flags.add_speed_limit) {
    rulebook->set_speed_limit(CreateSpeedLimitRule());
  }
  if (build_flags.add_discrete_value_rule) {
    rulebook->set_discrete_value_rule(CreateDiscreteValueRule());
  }
  if (build_flags.add_range_value_rule) {
    rulebook->set_range_value_rule(CreateRangeValueRule());
  }
  return std::move(rulebook);
}

std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook(const RoadRulebookRelatedRulesBuildFlags& build_flags) {
  auto rulebook = std::make_unique<MockRoadRulebook>();
  if (build_flags.roadrulebook_flags.add_right_of_way) {
    rulebook->set_right_of_way(CreateRightOfWayRule(build_flags.roadrulebook_flags.right_of_way_build_flags));
  }
  if (build_flags.roadrulebook_flags.add_direction_usage) {
    rulebook->set_direction_usage(CreateDirectionUsageRule());
  }
  if (build_flags.roadrulebook_flags.add_speed_limit) {
    rulebook->set_speed_limit(CreateSpeedLimitRule());
  }
  if (build_flags.roadrulebook_flags.add_discrete_value_rule) {
    rulebook->set_discrete_value_rule(
        CreateDiscreteValueRule(build_flags.consistent_related_rule_in_discrete_value_rule));
  }
  if (build_flags.roadrulebook_flags.add_range_value_rule) {
    rulebook->set_range_value_rule(CreateRangeValueRule(build_flags.consistent_related_rule_in_range_value_rule));
  }
  return std::move(rulebook);
}

std::unique_ptr<BulbGroup> CreateBulbGroup(bool add_missing_bulb_group) {
  const BulbGroup::Id bulb_group_id{add_missing_bulb_group ? "MissingBulbGroupId" : "BulbGroupId"};
  std::vector<std::unique_ptr<Bulb>> bulbs;
  bulbs.push_back(std::make_unique<Bulb>(Bulb::Id{"BulbId"}, InertialPosition(), Rotation(), rules::BulbColor::kRed,
                                         rules::BulbType::kRound, std::nullopt /* arrow_orientation_rad */,
                                         std::vector<rules::BulbState>{rules::BulbState::kOn}));
  return std::make_unique<BulbGroup>(bulb_group_id, InertialPosition(), Rotation(), std::move(bulbs));
}

std::unique_ptr<TrafficLight> CreateTrafficLight(const TrafficLightBuildFlags& build_flags) {
  const TrafficLight::Id id(build_flags.add_missing_traffic_light ? "MissingTrafficLightId" : "TrafficLightId");
  std::vector<std::unique_ptr<BulbGroup>> bulb_groups;
  bulb_groups.push_back(CreateBulbGroup(build_flags.add_missing_bulb_group));
  return std::make_unique<TrafficLight>(id, InertialPosition(), Rotation(), std::move(bulb_groups));
}

std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook() {
  return CreateTrafficLightBook(TrafficLightBookBuildFlags{});
}

std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook(const TrafficLightBookBuildFlags& build_flags) {
  auto traffic_light_book = std::make_unique<MockTrafficLightBook>();
  if (build_flags.add_traffic_light) {
    traffic_light_book->set_traffic_light(CreateTrafficLight(build_flags.traffic_light_book_flags));
  }
  return std::move(traffic_light_book);
}

std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook() { return std::make_unique<MockPhaseRingBook>(); }

std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook(const PhaseBuildFlags& build_flags) {
  auto phase_ring_book = std::make_unique<MockPhaseRingBook>();
  phase_ring_book->SetPhaseRing(CreatePhaseRing(build_flags));
  return phase_ring_book;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::unique_ptr<rules::RightOfWayRuleStateProvider> CreateRightOfWayRuleStateProvider() {
  return std::make_unique<MockRightOfWayRuleStateProvider>();
}
#pragma GCC diagnostic pop

std::unique_ptr<rules::PhaseProvider> CreatePhaseProvider() { return std::make_unique<MockPhaseProvider>(); }

std::unique_ptr<IntersectionBook> CreateIntersectionBook() { return std::make_unique<MockIntersectionBook>(); }

std::unique_ptr<rules::RuleRegistry> CreateRuleRegistry() { return std::make_unique<rules::RuleRegistry>(); }

std::unique_ptr<rules::RuleRegistry> CreateBasicRuleRegistry() {
  std::unique_ptr<rules::RuleRegistry> rule_registry = std::make_unique<rules::RuleRegistry>();
  const std::vector<DiscreteValueRule::DiscreteValue> discrete_values{
      {Rule::State::kStrict, Rule::RelatedRules{{"Yield Group", {}}, {"Vehicle Stop In Zone Behavior", {}}},
       Rule::RelatedUniqueIds{{"Bulb Group", {}}}, "Go"},
      {Rule::State::kStrict, Rule::RelatedRules{{"Yield Group", {}}}, Rule::RelatedUniqueIds{{"Bulb Group", {}}},
       "Stop"}};
  rule_registry->RegisterDiscreteValueRule(Rule::TypeId("Right-Of-Way Rule Type"), discrete_values);
  const std::vector<RangeValueRule::Range> range_values{
      {Rule::State::kStrict, Rule::RelatedRules{{"Yield Group", {}}, {"Vehicle Stop In Zone Behavior", {}}},
       Rule::RelatedUniqueIds{{"Bulb Group", {}}}, "Interstate highway - day time", 16.6, 27.8}};
  rule_registry->RegisterRangeValueRule(Rule::TypeId("Speed-Limit Rule Type"), range_values);
  return rule_registry;
}

std::unique_ptr<rules::DiscreteValueRuleStateProvider> CreateDiscreteValueRuleStateProvider() {
  return std::make_unique<MockDiscreteValueRuleStateProvider>();
}

std::unique_ptr<rules::RangeValueRuleStateProvider> CreateRangeValueRuleStateProvider() {
  return std::make_unique<MockRangeValueRuleStateProvider>();
}

}  // namespace test
}  // namespace api
}  // namespace maliput
