#pragma once

#include <memory>

#include "maliput/api/intersection.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule_state_provider.h"

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

/// Returns a rules::RightOfWayRule containing arbitrary state.
rules::RightOfWayRule Rule();

/// Returns a rules::DirectionUsageRule::State containing an arbitrary state.
rules::DirectionUsageRule::State CreateDirectionUsageRuleState();

/// Returns a rules::DirectionUsageRule containing an arbitrary state.
rules::DirectionUsageRule CreateDirectionUsageRule();

/// Returns an arbitrary RoadGeometry.
std::unique_ptr<RoadGeometry> CreateRoadGeometry();

/// Returns an arbitrary rules::RoadRulebook.
std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook();

/// Returns an arbitrary rules::PhaseRingBook.
std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook();

/// Returns an arbitrary rules::RuleStateProvider.
std::unique_ptr<rules::RuleStateProvider> CreateRuleStateProvider();

/// Returns an arbitrary rules::PhaseProvider.
std::unique_ptr<rules::PhaseProvider> CreatePhaseProvider();

/// Returns an arbitrary Intersection.
std::unique_ptr<Intersection> CreateIntersection(
    const Intersection::Id& id, const rules::PhaseRing::Id& ring_id);

}  // namespace test
}  // namespace api
}  // namespace maliput
