#pragma once

#include <memory>

#include "maliput/api/intersection_book.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule_state_provider.h"
#include "maliput/api/rules/traffic_light_book.h"

namespace maliput {
namespace api {
namespace test {

/// Returns a rules::LaneSRoute containing an arbitrary route.
rules::LaneSRoute CreateLaneSRoute();

/// Returns a rules::LaneSRange containing an arbitrary range.
rules::LaneSRange CreateLaneSRange();

/// Returns a rules::RightOfWayRule::State::YieldGroup of size two.
rules::RightOfWayRule::State::YieldGroup YieldGroup2();

/// Returns a rules::RightOfWayRule::State containing no yield groups.
rules::RightOfWayRule::State NoYieldState();

/// Returns a rules::RightOfWayRule::State containing yield groups.
rules::RightOfWayRule::State YieldState();

///  Returns an arbitrary rules::RightOfWayRule::RelatedBulbGroups.
rules::RightOfWayRule::RelatedBulbGroups RelatedBulbGroups();

/// Returns a rules::RightOfWayRule containing arbitrary state.
rules::RightOfWayRule CreateRightOfWayRule();

/// Returns a rules::DirectionUsageRule::State containing an arbitrary state.
rules::DirectionUsageRule::State CreateDirectionUsageRuleState();

/// Returns a rules::DirectionUsageRule containing an arbitrary state.
rules::DirectionUsageRule CreateDirectionUsageRule();

/// Returns a rules::DiscreteValueRule containing an arbitrary state.
rules::DiscreteValueRule CreateDiscreteValueRule();

/// Returns a rules::RangeValueRule::Range.
rules::RangeValueRule::Range CreateRange();

/// Returns a rules::RangeValueRule containing an arbitrary state.
rules::RangeValueRule CreateRangeValueRule();

/// Returns an arbitrary RoadGeometry.
std::unique_ptr<RoadGeometry> CreateRoadGeometry();

/// Holds RoadGeometry build flag configuration.
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
std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook();

/// Returns an arbitrary rules::TrafficLightBook.
std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook();

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
