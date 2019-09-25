#include "maliput/api/road_network.h"

#include <algorithm>
#include <utility>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {

RoadNetwork::RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry,
                         std::unique_ptr<const rules::RoadRulebook> rulebook,
                         std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
                         std::unique_ptr<IntersectionBook> intersection_book,
                         std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
                         std::unique_ptr<rules::RightOfWayRuleStateProvider> right_of_way_rule_state_provider,
                         std::unique_ptr<rules::PhaseProvider> phase_provider,
                         std::unique_ptr<rules::RuleRegistry> rule_registry,
                         std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider,
                         std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider)
    : road_geometry_(std::move(road_geometry)),
      rulebook_(std::move(rulebook)),
      traffic_light_book_(std::move(traffic_light_book)),
      intersection_book_(std::move(intersection_book)),
      phase_ring_book_(std::move(phase_ring_book)),
      right_of_way_rule_state_provider_(std::move(right_of_way_rule_state_provider)),
      phase_provider_(std::move(phase_provider)),
      rule_registry_(std::move(rule_registry)),
      discrete_value_rule_state_provider_(std::move(discrete_value_rule_state_provider)),
      range_value_rule_state_provider_(std::move(range_value_rule_state_provider)) {
  MALIPUT_THROW_UNLESS(road_geometry_.get() != nullptr);
  MALIPUT_THROW_UNLESS(rulebook_.get() != nullptr);
  MALIPUT_THROW_UNLESS(traffic_light_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(intersection_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(phase_ring_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(right_of_way_rule_state_provider_.get() != nullptr);
  MALIPUT_THROW_UNLESS(phase_provider_.get() != nullptr);
  MALIPUT_THROW_UNLESS(rule_registry_.get() != nullptr);
  MALIPUT_THROW_UNLESS(discrete_value_rule_state_provider_.get() != nullptr);
  MALIPUT_THROW_UNLESS(range_value_rule_state_provider_.get() != nullptr);
}

drake::optional<RoadPositionResult> RoadNetwork::ToRoadPosition(
    const GeoPosition& geo_position, const RoadPosition* hint,
    const std::function<bool(const RoadPositionResult&)>& filter) const {
  RoadPositionResult result;
  result.road_position =
      road_geometry_->ToRoadPosition(geo_position, hint, &(result.nearest_position), &(result.distance));
  if (filter(result)) {
    return result;
  }
  return drake::nullopt;
}

std::vector<RoadPositionResult> RoadNetwork::FindRoadPositions(
    const GeoPosition& geo_position, double radius,
    const std::function<bool(const RoadPositionResult&)>& filter) const {
  std::vector<RoadPositionResult> result = road_geometry_->FindRoadPositions(geo_position, radius);
  // TODO(agalbachicar)   If this code is ever moved to C++17, use std::not_fn() instead.
  result.erase(std::remove_if(result.begin(), result.end(), std::not1(filter)), result.end());
  return result;
}

}  // namespace api
}  // namespace maliput
