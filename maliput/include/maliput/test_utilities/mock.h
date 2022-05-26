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
#pragma once

#include <memory>

#include "maliput/api/intersection_book.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/road_network.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/range_value_rule_state_provider.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule_registry.h"
#include "maliput/api/rules/traffic_light_book.h"

namespace maliput {
namespace api {
namespace test {

struct RoadGeometryIdIndexBuildFlags {
  bool add_branchpoint{false};
  bool add_junction{false};
  bool add_lane{false};
  bool add_segment{false};
};

/// Holds RoadGeometry build configuration.
/// @see CreateRoadGeometry() docstring for full details on how this
///      structure pairs with the function.
struct RoadGeometryBuildFlags {
  bool add_junction{false};
  bool add_segment{false};
  bool add_lane{false};
  bool add_branchpoint{false};
  bool add_lane_end_set{false};
  bool expects_throw{false};
  RoadGeometryIdIndexBuildFlags id_index_build_flags{};
};

/// Holds RoadGeometry contiguity build configuration.
/// @see CreateMockContiguousRoadGeometry() docstring for full details on how this
///      structure pairs with the function.
struct RoadGeometryContiguityBuildFlags {
  bool add_linear_mismatch{false};
  bool add_angular_mismatch{false};
  double linear_tolerance{0};
  double angular_tolerance{0};
  math::Vector3 inertial_to_backend_frame_translation{0., 0., 0.};
};

/// Holds RoadRulebook contiguity build configuration.
/// @see CreateMockContiguousRoadRulebook() docstring for full details on how this
///      structure pairs with the function.
struct RoadRulebookContiguityBuildFlags {
  bool add_discrete_value_rule{false};
  bool add_range_value_rule{false};
};

/// Holds RoadNetwork contiguity build configuration.
/// @see CreateMockContiguousRoadGeometry() and CreateMockContiguousRoadRulebook() docstring
///      for full details on how this structure pairs with the function
struct RoadNetworkContiguityBuildFlags {
  RoadGeometryContiguityBuildFlags rg_contiguity_build_flags{};
  RoadRulebookContiguityBuildFlags rulebook_contiguity_build_flags{};
  bool expects_throw{false};
};

/// Holds RightOfWayRule build configurations.
/// @see CreateRightOfWayRule() docstring for full details on
///      how this structure pairs with the function.
struct RightOfWayBuildFlags {
  bool add_related_bulb_groups{true};
};

/// Holds RoadRulebook build configurations.
/// @see CreateRoadRulebook() docstring for full details on
///      how this structure pairs with the function.
struct RoadRulebookBuildFlags {
  bool add_right_of_way{true};
  RightOfWayBuildFlags right_of_way_build_flags{};
  bool add_direction_usage{false};
  bool add_speed_limit{true};
  bool add_discrete_value_rule{false};
  bool add_range_value_rule{false};
};

/// Holds RoadRulebook build configurations when RelatedRules consitency are
/// under test.
/// @see CreateRoadRulebook() docstring for full details on
///      how this structure pairs with the function.
struct RoadRulebookRelatedRulesBuildFlags {
  RoadRulebookBuildFlags roadrulebook_flags;
  bool consistent_related_rule_in_discrete_value_rule{false};
  bool consistent_related_rule_in_range_value_rule{false};
};

/// Holds TrafficLight build configurations.
/// @see CreateTrafficLight() docstring for full details on
///      how this structure pairs with the function.
struct TrafficLightBuildFlags {
  bool add_missing_traffic_light{false};
  bool add_missing_bulb_group{false};
};

/// Holds TrafficLightBook build configurations.
/// @see CreateTrafficLightBook() docstring for full details on
///      how this structure pairs with the function.
struct TrafficLightBookBuildFlags {
  bool add_traffic_light{false};
  TrafficLightBuildFlags traffic_light_book_flags{};
};

/// Holds Phase build configurations.
/// @see CreatePhase() docstring for full details on how this structure pairs
///      with the function.
struct PhaseBuildFlags {
  bool add_missing_rule{false};
  bool add_missing_value{false};
  bool add_missing_bulb{false};
  bool add_missing_bulb_state{false};
};

class MockLaneEndSet final : public LaneEndSet {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockLaneEndSet)
  MockLaneEndSet() = default;
  void set_lane_end(const LaneEnd& lane_end) { lane_end_ = lane_end; }

 private:
  int do_size() const override { return 1; }
  const LaneEnd& do_get(int index) const override {
    MALIPUT_THROW_UNLESS(index != 0);
    return lane_end_;
  }

  LaneEnd lane_end_;
};

class MockBranchPoint final : public BranchPoint {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockBranchPoint)
  explicit MockBranchPoint(const BranchPointId& id) : BranchPoint(), id_(id) {}
  void set_road_geometry(RoadGeometry* road_geometry) { road_geometry_ = road_geometry; }
  void set_lane_end_set_a(std::unique_ptr<MockLaneEndSet> lane_end_set_a) {
    lane_end_set_a_ = std::move(lane_end_set_a);
  }
  void set_lane_end_set_b(std::unique_ptr<MockLaneEndSet> lane_end_set_b) {
    lane_end_set_b_ = std::move(lane_end_set_b);
  }

 private:
  BranchPointId do_id() const override { return id_; }
  const RoadGeometry* do_road_geometry() const override { return road_geometry_; }
  const LaneEndSet* DoGetConfluentBranches(const LaneEnd&) const override { return nullptr; }
  const LaneEndSet* DoGetOngoingBranches(const LaneEnd&) const override { return nullptr; }
  std::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd&) const override { return std::nullopt; }
  const LaneEndSet* DoGetASide() const override { return lane_end_set_a_.get(); }
  const LaneEndSet* DoGetBSide() const override { return lane_end_set_b_.get(); }

  BranchPointId id_;
  RoadGeometry* road_geometry_{};
  std::unique_ptr<MockLaneEndSet> lane_end_set_a_;
  std::unique_ptr<MockLaneEndSet> lane_end_set_b_;
};

class MockLane final : public Lane {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockLane);
  MockLane(const LaneId& id) : Lane(), id_(id) {}
  MockLane(const LaneId& id, const InertialPosition& start_ip, const Rotation& start_rot,
           const InertialPosition& end_ip, const Rotation& end_rot)
      : Lane(), id_(id), start_ip_(start_ip), start_rot_(start_rot), end_ip_(end_ip), end_rot_(end_rot) {}
  MockLane(const LaneId& id, const LanePositionResult& lane_position_result)
      : Lane(), id_(id), lane_position_result_(lane_position_result) {}
  void set_segment(Segment* segment) { segment_ = segment; }
  void set_start_bp(BranchPoint* start_bp) { start_bp_ = start_bp; }
  void set_end_bp(BranchPoint* end_bp) { end_bp_ = end_bp; }

 private:
  LaneId do_id() const override { return id_; };
  const Segment* do_segment() const override { return segment_; };
  int do_index() const override { return 0; };
  const Lane* do_to_left() const override { return nullptr; };
  const Lane* do_to_right() const override { return nullptr; };
  double do_length() const override { return 100; };
  RBounds do_lane_bounds(double) const override { return RBounds(-1, 1); };
  RBounds do_segment_bounds(double) const override { return RBounds(-1, 1); };
  HBounds do_elevation_bounds(double, double) const override { return HBounds(0, 10); };
  InertialPosition DoToInertialPosition(const LanePosition& lane_pos) const override {
    return lane_pos.s() ? end_ip_ : start_ip_;
  }
  LanePositionResult DoToLanePosition(const InertialPosition&) const override { return lane_position_result_; }
  Rotation DoGetOrientation(const LanePosition& lane_pos) const override {
    return lane_pos.s() ? end_rot_ : start_rot_;
  }
  LanePosition DoEvalMotionDerivatives(const LanePosition&, const IsoLaneVelocity&) const override {
    return LanePosition(0, 0, 0);
  }
  const BranchPoint* DoGetBranchPoint(const LaneEnd::Which end) const override {
    return end == LaneEnd::Which::kStart ? start_bp_ : end_bp_;
  };
  const LaneEndSet* DoGetConfluentBranches(const LaneEnd::Which) const override { return nullptr; }
  const LaneEndSet* DoGetOngoingBranches(const LaneEnd::Which) const override { return nullptr; }
  std::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd::Which) const override { return std::nullopt; }

  LaneId id_;
  Segment* segment_{};
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};
  InertialPosition start_ip_{};
  Rotation start_rot_{};
  InertialPosition end_ip_{};
  Rotation end_rot_{};
  LanePositionResult lane_position_result_{};
};

class MockSegment final : public Segment {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockSegment)
  MockSegment(const SegmentId& id) : Segment(), id_(id) {}
  void set_junction(Junction* junction) { junction_ = junction; }
  void set_lane(std::unique_ptr<MockLane> lane) { lane_ = std::move(lane); }

 private:
  SegmentId do_id() const override { return id_; }
  const Junction* do_junction() const override { return junction_; }
  int do_num_lanes() const override { return lane_ != nullptr ? 1 : 0; }
  const Lane* do_lane(int index) const override {
    MALIPUT_THROW_UNLESS(index == 0);
    return lane_.get();
  }

  SegmentId id_;
  Junction* junction_{};
  std::unique_ptr<MockLane> lane_;
};

class MockJunction final : public Junction {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockJunction)
  MockJunction(const JunctionId& id) : Junction(), id_(id) {}
  void set_road_geometry(RoadGeometry* road_geometry) { road_geometry_ = road_geometry; }
  void set_segment(std::unique_ptr<MockSegment> segment) { segment_ = std::move(segment); }

 private:
  JunctionId do_id() const override { return JunctionId("mock"); }
  int do_num_segments() const override { return 1; }
  const Segment* do_segment(int i) const override {
    MALIPUT_THROW_UNLESS(i == 0);
    return segment_.get();
  }
  const RoadGeometry* do_road_geometry() const override { return road_geometry_; }

  JunctionId id_;
  RoadGeometry* road_geometry_{};
  std::unique_ptr<MockSegment> segment_;
};

class MockIdIndex final : public RoadGeometry::IdIndex {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockIdIndex);
  MockIdIndex() = default;
  void add_junction_to_map(const JunctionId& id, const Junction* junction) { junction_map_.emplace(id, junction); }
  void add_lane_to_map(const LaneId& id, const Lane* lane) { lane_map_.emplace(id, lane); }
  void add_segment_to_map(const SegmentId& id, const Segment* segment) { segment_map_.emplace(id, segment); }
  void add_branchpoint_to_map(const BranchPointId& id, const BranchPoint* branch_point) {
    branch_point_map_.emplace(id, branch_point);
  }

 private:
  const BranchPoint* DoGetBranchPoint(const BranchPointId& branch_point_id) const override {
    const auto it = branch_point_map_.find(branch_point_id);
    return (it == branch_point_map_.end()) ? nullptr : it->second;
  }
  const Junction* DoGetJunction(const JunctionId& junction_id) const override {
    const auto it = junction_map_.find(junction_id);
    return (it == junction_map_.end()) ? nullptr : it->second;
  }
  const Lane* DoGetLane(const LaneId& lane_id) const override {
    const auto it = lane_map_.find(lane_id);
    return (it == lane_map_.end()) ? nullptr : it->second;
  }
  const Segment* DoGetSegment(const SegmentId& segment_id) const override {
    const auto it = segment_map_.find(segment_id);
    return (it == segment_map_.end()) ? nullptr : it->second;
  }
  const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override { return lane_map_; }

  std::unordered_map<BranchPointId, const BranchPoint*> branch_point_map_;
  std::unordered_map<JunctionId, const Junction*> junction_map_;
  std::unordered_map<LaneId, const Lane*> lane_map_;
  std::unordered_map<SegmentId, const Segment*> segment_map_;
};

class MockRoadGeometry : public RoadGeometry {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadGeometry)
  MockRoadGeometry(const RoadGeometryId& id) : id_(id) {}
  MockRoadGeometry(const RoadGeometryId& id, const double& linear_tolerance, const double& angular_tolerance,
                   const math::Vector3& inertial_to_backend_frame_translation)
      : id_(id),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance),
        inertial_to_backend_frame_translation_(inertial_to_backend_frame_translation) {}

  void add_junction(std::unique_ptr<MockJunction> junction) { junctions_.push_back(std::move(junction)); }
  void set_start_bp(std::unique_ptr<MockBranchPoint> start_bp) { start_bp_ = std::move(start_bp); }
  void set_end_bp(std::unique_ptr<MockBranchPoint> end_bp) { end_bp_ = std::move(end_bp); }

  MockBranchPoint* start_bp() { return start_bp_.get(); }
  MockBranchPoint* end_bp() { return end_bp_.get(); }
  MockIdIndex* GetIdIndex() { return &mock_id_index_; }

 private:
  RoadGeometryId do_id() const override { return id_; }
  int do_num_junctions() const override { return junctions_.size(); }
  const Junction* do_junction(int i) const override {
    MALIPUT_THROW_UNLESS(i < static_cast<int>(junctions_.size()));
    return junctions_[i].get();
  }
  int do_num_branch_points() const override {
    if (start_bp_ != nullptr && end_bp_ != nullptr) {
      return 2;
    } else if (start_bp_ == nullptr && end_bp_ == nullptr) {
      return 0;
    }
    return 1;
  }
  const BranchPoint* do_branch_point(int i) const override {
    MALIPUT_THROW_UNLESS(i == 0 || i == 1);
    return i == 0 ? start_bp_.get() : end_bp_.get();
  }
  const IdIndex& DoById() const override { return mock_id_index_; }
  api::RoadPositionResult DoToRoadPosition(const InertialPosition&, const std::optional<RoadPosition>&) const override {
    return RoadPositionResult();
  }
  std::vector<api::RoadPositionResult> DoFindRoadPositions(const InertialPosition&, double) const override {
    return {{RoadPositionResult()}};
  }
  double do_linear_tolerance() const override { return linear_tolerance_; }
  double do_angular_tolerance() const override { return angular_tolerance_; }
  double do_scale_length() const override { return 0; }
  math::Vector3 do_inertial_to_backend_frame_translation() const override {
    return inertial_to_backend_frame_translation_;
  }

  MockIdIndex mock_id_index_;
  RoadGeometryId id_;
  double linear_tolerance_{0};
  double angular_tolerance_{0};
  math::Vector3 inertial_to_backend_frame_translation_{};
  std::vector<std::unique_ptr<MockJunction>> junctions_;
  std::unique_ptr<MockBranchPoint> start_bp_;
  std::unique_ptr<MockBranchPoint> end_bp_;
};

/// Returns a LaneSRoute containing an arbitrary route.
LaneSRoute CreateLaneSRoute();

/// Returns a LaneSRange containing an arbitrary range.
LaneSRange CreateLaneSRange();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Returns a rules::RightOfWayRule::State::YieldGroup of size two.
rules::RightOfWayRule::State::YieldGroup YieldGroup2();

/// Returns a rules::RightOfWayRule::State containing no yield groups.
rules::RightOfWayRule::State NoYieldState();

/// Returns a rules::RightOfWayRule::State containing yield groups.
rules::RightOfWayRule::State YieldState();

///  Returns an arbitrary rules::RightOfWayRule::RelatedBulbGroups.
rules::RightOfWayRule::RelatedBulbGroups RelatedBulbGroups();

/// Returns a rules::RightOfWayRule containing arbitrary state.
///
/// Forwards the call to CreateRightOfWayRule() with a default
/// initialized RightOfWayBuildFlags.
rules::RightOfWayRule CreateRightOfWayRule();

/// Returns a rules::RightOfWayRule containing arbitrary state.
///
/// When `build_flags.add_related_bulb_groups` is true, the rule is
/// constructed with RelatedBulbGroups() result. Otherwise, the rule
/// is constructed with an empty collection for the field.
rules::RightOfWayRule CreateRightOfWayRule(const RightOfWayBuildFlags& build_flags);

/// Returns a rules::SpeedLimitRule containing an arbitrary state.
rules::SpeedLimitRule CreateSpeedLimitRule();

/// Returns an arbitrary rules::DirectionUsageRule::State.
rules::DirectionUsageRule::State CreateDirectionUsageRuleState();

/// Returns a rules::DirectionUsageRule containing an arbitrary state.
rules::DirectionUsageRule CreateDirectionUsageRule();
#pragma GCC diagnostic pop

/// Returns an empty rules::Rule::RelatedRules.
rules::Rule::RelatedRules CreateEmptyRelatedRules();

/// Returns an arbitrary rules::Rule::RelatedRules.
rules::Rule::RelatedRules CreateNonEmptyRelatedRules();

/// Returns an empty rules::Rule::RelatedUniqueIds.
rules::Rule::RelatedUniqueIds CreateEmptyRelatedUniqueIds();

/// Returns an arbitrary rules::Rule::RelatedUniqueIds.
rules::Rule::RelatedUniqueIds CreateNonEmptyRelatedUniqueIds();

/// Returns a rules::DiscreteValueRule containing an arbitrary state.
rules::DiscreteValueRule CreateDiscreteValueRule();

/// Returns a rules::DiscreteValueRule containing an arbitrary state.
rules::DiscreteValueRule CreateDiscreteValueRuleForContiguityTest();

/// Returns a rules::RangeValueRule::Range.
rules::RangeValueRule::Range CreateRange();

/// Returns a rules::RangeValueRule containing an arbitrary state.
rules::RangeValueRule CreateRangeValueRule();

/// Returns an arbitrary rules::Phase.
rules::Phase CreatePhase();

/// Returns a rules::Phase based on `build_flags` configuration.
///
/// When `build_flags.add_missing_rule` is true, an unknown rule id is added;
/// and if `build_flags.add_missing_value` is true, an unknown
/// rules::DiscreteValueRule::DiscreteValue is added. Otherwise, it adds a
/// reference to both rules::Rule::Id and
/// rules::DiscreteValueRule::DiscreteValue offered by
/// CreateDiscreteValueRule(). When `build_flags.add_missing_bulb` is true, an
/// unknown rules::UniqueBulbId is added; and if
/// `build_flags.add_missing_bulb_state` is true, the rules::Bulb's
/// rules::BulbState will be other than the one set by CreateBulbGroup().
rules::Phase CreatePhase(const PhaseBuildFlags& build_flags);

/// Returns an arbitrary rules::PhaseRing whose rules::Phase is the result of
/// CreatePhase().
rules::PhaseRing CreatePhaseRing();

/// Returns a rules::PhaseRing whose rules::Phase is the result of
/// `CreatePhase(build_flags)`.
rules::PhaseRing CreatePhaseRing(const PhaseBuildFlags& build_flags);

/// Returns a rules::RangeValueRule containing an arbitrary state.
rules::RangeValueRule CreateRangeValueRuleForContiguityTest();

/// Returns an arbitrary RoadNetwork.
///
/// It calls all the Create*() functions in this header file to populate the RoadNetwork.
std::unique_ptr<RoadNetwork> CreateRoadNetwork();

/// Returns an arbitrary RoadGeometry.
std::unique_ptr<RoadGeometry> CreateRoadGeometry();

// Builds a RoadGeometry parametrically based on `build_flags` configuration.
//
// When `build_flags.add_junction` is true, an empty Junction is added; and
// if `build_flags.add_segment` is true, an empty Segment is added to the
// Junction; and  if `build_flags.add_lane` is true, an empty Lane is added
// to the Segment.
// When `build_flags.add_branchpoint` is true, two BranchPoints are added to
// the RoadGeometry.
// When all items in `build_flags` are true, respective LaneEndSets are
// created to link Lane with BranchPoints.
//
// @returns A MockRoadGeometry pointer.
std::unique_ptr<RoadGeometry> CreateRoadGeometry(const RoadGeometryBuildFlags& build_flags);

/// Builds a RoadGeometry based on `build_flags` configuration.
///
/// When `build.flags.add_linear_mismatch` is true, end of first lane and start of second lane will be
/// separated by more than linear_tolerance.
/// When `build_flags.add_angular_mismatch` is true, heading angular distance
/// between the end of the first lane and the start of the second lane will be bigger than
/// angular_tolerance.
///
/// @returns A MockRoadGeometry pointer.
std::unique_ptr<RoadGeometry> CreateMockContiguousRoadGeometry(const RoadGeometryContiguityBuildFlags& build_flags);

/// Returns an arbitrary one-lane RoadGeometry.
std::unique_ptr<RoadGeometry> CreateOneLaneRoadGeometry();

/// Returns an arbitrary two-lane RoadGeometry.
///
/// Forwards the call to CreateTwoLanesRoadGeometry(const LanePositionResult&, const LanePositionResult&)
/// and provides two hardcoded LanePositionResults:
/// - "lane_a": {{10., 20., 30.}, {12., 89., 1.}, 0.5}
/// - "lane_b": {{40., 50., 60.}, {50., 1., 45.}, 30.}
std::unique_ptr<RoadGeometry> CreateTwoLanesRoadGeometry();

/// Returns an arbitrary two-lane RoadGeometry and each lane will return @p lane_a_pos_result and
/// @p lane_b_pos_result when calling Lane::ToLanePosition() with any InertialPosition.
std::unique_ptr<RoadGeometry> CreateTwoLanesRoadGeometry(const LanePositionResult& lane_a_pos_result,
                                                         const LanePositionResult& lane_b_pos_result);

/// Returns an aribtrary lane with @p id .
std::unique_ptr<Lane> CreateLane(const LaneId& id);

/// Returns an arbitrary rules::RoadRulebook.
///
/// Forwards the call to CreateRoadRulebook() passing a default-constructed
/// RoadRulebookBuildFlags structure.
std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook();

/// Returns an arbitrary rules::RoadRulebook.
///
/// When `build_flags.add_right_of_way` is true, a RightOfWayRule is
/// created with CreateRightOfWayRule() and `build_flags.right_of_way_build_flags`
/// is used to configure the RightOfWayRule.
/// When `build_flags.add_direction_usage` is true, a DirectionUsageRule
/// is created with CreateDirectionUsageRule().
/// When `build_flags.add_speed_limit` is true, a SpeedLimitRule is created
/// with CreateSpeedLimitRule.
/// When `build_flags.add_discrete_value` is true, a DiscreteValueRule
/// is created with CreateDiscreteValueRule().
/// When `build_flags.add_range_value` is true, a RangeValueRule
/// is created with CreateRangeValueRule().
std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook(const RoadRulebookBuildFlags& build_flags);

/// Returns an arbitrary rules::RoadRulebook.
///
/// @see CreateRoadRulebook(const RoadRulebookBuildFlags& build_flags) for
/// rule construction logic.
///
/// When `build_flags.roadrulebook.add_discrete_value`,
/// `build_flags.consistent_related_rule_in_discrete_value_rule` will create a
/// self reference to provide a valid rule::Rule::Id as a RelatedRule.
/// When `build_flags.roadrulebook.add_Range_value`,
/// `build_flags.consistent_related_rule_in_range_value_rule` will create a
/// self reference to provide a valid rule::Rule::Id as a RelatedRule.
std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook(const RoadRulebookRelatedRulesBuildFlags& build_flags);

/// Returns an arbitrary rules::RoadRulebook.
///
/// When `build_flags.add_discrete_value` is true, a DiscreteValueRule
/// is created with CreateDiscreteValueRuleForContiguityTest().
/// When `build_flags.add_range_value` is true, a RangeValueRule
/// is created with CreateRangeValueRuleForContiguityTest().
std::unique_ptr<rules::RoadRulebook> CreateMockContiguousRoadRulebook(
    const RoadRulebookContiguityBuildFlags& build_flags);

/// Returns an arbitrary rules::BulbGroup.
///
/// When `add_missing_bulb_group` is true, the rules::BulbGroup's ID is
/// "MissingBulbGroupId". Otherwise, ID is "BulbGroupId".
/// `traffic_light_id` is the rules::TrafficLight::Id to set to the
/// rules::Bulb's rules::UniqueBulbId.
std::unique_ptr<rules::BulbGroup> CreateBulbGroup(bool add_missing_bulb_group);

/// Returns an arbitrary rules::TrafficLight.
///
/// When `build_flags.add_missing_bulb_group` is true, the
/// rules::TrafficLight's ID is "MissingTrafficLightId". Otherwise, ID
/// is "TrafficLightId".
/// BulbGroup creation is done via
/// `CreateBulbGroup(build_flags.add_missing_bulb_group)`.
std::unique_ptr<rules::TrafficLight> CreateTrafficLight(const TrafficLightBuildFlags& build_flags);

/// Returns an arbitrary rules::TrafficLightBook.
std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook();

/// Returns an arbitrary rules::TrafficLightBook.
std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook(const TrafficLightBookBuildFlags& build_flags);

/// Returns an arbitrary rules::PhaseRingBook.
std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook();

/// Returns an arbitrary rules::PhaseRingBook based on `build_flags`.
std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook(const PhaseBuildFlags& build_flags);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Returns an arbitrary rules::RightOfWayRuleStateProvider.
std::unique_ptr<rules::RightOfWayRuleStateProvider> CreateRightOfWayRuleStateProvider();
#pragma GCC diagnostic pop

/// Returns an arbitrary rules::PhaseProvider.
std::unique_ptr<rules::PhaseProvider> CreatePhaseProvider();

/// Returns an arbitrary IntersectionBook.
std::unique_ptr<IntersectionBook> CreateIntersectionBook();

/// Returns an arbitrary rules::RuleRegistry.
std::unique_ptr<rules::RuleRegistry> CreateRuleRegistry();

/// Returns an arbitrary rules::RuleRegistry populated with discrete and range value rule types.
std::unique_ptr<rules::RuleRegistry> CreateBasicRuleRegistry();

/// Returns an arbitrary rules::DiscreteValueRuleStateProvider.
std::unique_ptr<rules::DiscreteValueRuleStateProvider> CreateDiscreteValueRuleStateProvider();

/// Returns an arbitrary rules::RangeValueRuleStateProvider.
std::unique_ptr<rules::RangeValueRuleStateProvider> CreateRangeValueRuleStateProvider();

}  // namespace test
}  // namespace api
}  // namespace maliput
