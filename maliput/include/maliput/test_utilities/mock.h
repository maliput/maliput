#pragma once

#include <memory>

#include "maliput/api/intersection_book.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_geometry.h"
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

/// Returns a LaneSRoute containing an arbitrary route.
LaneSRoute CreateLaneSRoute();

/// Returns a LaneSRange containing an arbitrary range.
LaneSRange CreateLaneSRange();

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
std::unique_ptr<RoadGeometry> CreateTwoLanesRoadGeometry();

/// Returns a RoadGeometry filled with `lanes`.
/// The RoadGeometry is not completed. Its use is aimed to obtained the lanes by ById()->GetLane().
/// @param lanes Is a vector of lanes to be added to the RoadGeometry's IdIndex.
std::unique_ptr<RoadGeometry> CreateMultipleLanesRoadGeometry(const std::vector<Lane*>& lanes);

/// Returns a Lane.
/// @param id Id of the lane.
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

std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook(const PhaseBuildFlags& build_flags);

/// Returns an arbitrary rules::RightOfWayRuleStateProvider.
std::unique_ptr<rules::RightOfWayRuleStateProvider> CreateRightOfWayRuleStateProvider();

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
