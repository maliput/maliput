#pragma once

#include <memory>

#include "maliput/api/intersection_book.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule_state_provider.h"
#include "maliput/api/rules/traffic_light_book.h"

namespace maliput {
namespace api {
namespace test {

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

/// Returns a rules::DiscreteValueRule containing an arbitrary state.
rules::DiscreteValueRule CreateDiscreteValueRule();

/// Returns a rules::RangeValueRule::Range.
rules::RangeValueRule::Range CreateRange();

/// Returns a rules::RangeValueRule containing an arbitrary state.
rules::RangeValueRule CreateRangeValueRule();

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

/// Returns an arbitrary one-lane RoadGeometry.
std::unique_ptr<RoadGeometry> CreateOneLaneRoadGeometry();

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
std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook(const RoadRulebookBuildFlags& build_flags);

/// Returns an arbitrary rules::BulbGroup.
///
/// When `add_missing_bulb_group` is true, the rules::BulbGroup's ID is
/// "MissingBulbGroupId". Otherwise, ID is "BulbGroupId".
rules::BulbGroup CreateBulbGroup(bool add_missing_bulb_group);

/// Returns an arbitrary rules::TrafficLight.
///
/// When `build_flags.add_missing_bulb_group` is true, the
/// rules::TrafficLight's ID is "MissingTrafficLightId". Otherwise, ID
/// is "TrafficLightId".
/// BulbGroup creation is done via
/// `CreateBulbGroup(build_flags.add_missing_bulb_group)`.
rules::TrafficLight CreateTrafficLight(const TrafficLightBuildFlags& build_flags);

/// Returns an arbitrary rules::TrafficLightBook.
std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook();

/// Returns an arbitrary rules::TrafficLightBook.
std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook(const TrafficLightBookBuildFlags& build_flags);

/// Returns an arbitrary rules::PhaseRingBook.
std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook();

/// Returns an arbitrary rules::RuleStateProvider.
std::unique_ptr<rules::RuleStateProvider> CreateRuleStateProvider();

/// Returns an arbitrary rules::PhaseProvider.
std::unique_ptr<rules::PhaseProvider> CreatePhaseProvider();

/// Returns an arbitrary IntersectionBook.
std::unique_ptr<IntersectionBook> CreateIntersectionBook();

}  // namespace test
}  // namespace api
}  // namespace maliput
