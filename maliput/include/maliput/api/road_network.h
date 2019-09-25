#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "maliput/api/intersection_book.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/range_value_rule_state_provider.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule_registry.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/api/rules/traffic_light_book.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace maliput {
namespace api {

/// A container that aggregates everything pertaining to Maliput.
class RoadNetwork {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetwork)

  /// Constructs a RoadNetwork instance. After creation, you are encouraged to
  /// validate it using ValidateRoadNetwork(), which is defined in
  /// maliput/api/road_network_validator.h.
  RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry, std::unique_ptr<const rules::RoadRulebook> rulebook,
              std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
              std::unique_ptr<IntersectionBook> intersection_book,
              std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
              std::unique_ptr<rules::RightOfWayRuleStateProvider> right_of_way_rule_state_provider,
              std::unique_ptr<rules::PhaseProvider> phase_provider, std::unique_ptr<rules::RuleRegistry> rule_registry,
              std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider,
              std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider);

  virtual ~RoadNetwork() = default;

  const RoadGeometry* road_geometry() const { return road_geometry_.get(); }

  const rules::RoadRulebook* rulebook() const { return rulebook_.get(); }

  const rules::TrafficLightBook* traffic_light_book() const { return traffic_light_book_.get(); }

  IntersectionBook* intersection_book() const { return intersection_book_.get(); }

  const rules::PhaseRingBook* phase_ring_book() const { return phase_ring_book_.get(); }

  const rules::RightOfWayRuleStateProvider* right_of_way_rule_state_provider() const {
    return right_of_way_rule_state_provider_.get();
  }

  const rules::PhaseProvider* phase_provider() const { return phase_provider_.get(); }

  const rules::RuleRegistry* rule_registry() const { return rule_registry_.get(); }

  const rules::DiscreteValueRuleStateProvider* discrete_value_rule_state_provider() const {
    return discrete_value_rule_state_provider_.get();
  }

  const rules::RangeValueRuleStateProvider* range_value_rule_state_provider() const {
    return range_value_rule_state_provider_.get();
  }

  /// Forwards the call to `road_geometry()->ToRoadPosition()` and filters the
  /// result with `filter`.
  ///
  /// Note that `RoadGeometry::ToRoadPosition()` receives `nearest_position`
  /// and `distance` as output attributes. Both will be provided and in case
  /// `filter` returns true, be part of the return value.
  ///
  /// @param geo_position @see RoadGeometry::ToRoadPosition() for full
  /// parameter documentation.
  /// @param hint @see RoadGeometry::ToRoadPosition() for full parameter
  /// documentation.
  /// @param filter An arbitrary functor that determines whether or not the
  /// result of `road_geometry()->ToRoadPosition()` is valid or not.
  ///
  /// @returns The result of `road_geometry()->ToRoadPosition()` wrapped in
  /// a RoadPositionResult when `filter()` returns true. Otherwise, nullopt.
  drake::optional<RoadPositionResult> ToRoadPosition(
      const GeoPosition& geo_position, const RoadPosition* hint,
      const std::function<bool(const RoadPositionResult&)>& filter) const;

  /// Forwards the call to `road_geometry()->FindRoadPositions()` and filters
  /// the result with `filter`.
  ///
  /// @param geo_position @see RoadGeometry::FindRoadPositions() for full
  /// parameter documentation.
  /// @param radius @see RoadGeometry::FindRoadPositions() for full parameter
  /// documentation.
  /// @param filter An arbitrary functor that determines whether or not the
  /// results of `road_geometry()->FindRoadPositions()` are valid or not.
  ///
  /// @returns The result of `road_geometry()->FindRoadPositions()` filtered
  /// with `filter`. When `filter()` returns true, the RoadPositionResult is
  /// included in the returned vector.
  std::vector<RoadPositionResult> FindRoadPositions(const GeoPosition& geo_position, double radius,
                                                    const std::function<bool(const RoadPositionResult&)>& filter) const;

 private:
  std::unique_ptr<const RoadGeometry> road_geometry_;
  std::unique_ptr<const rules::RoadRulebook> rulebook_;
  std::unique_ptr<const rules::TrafficLightBook> traffic_light_book_;
  std::unique_ptr<IntersectionBook> intersection_book_;
  std::unique_ptr<rules::PhaseRingBook> phase_ring_book_;
  std::unique_ptr<rules::RightOfWayRuleStateProvider> right_of_way_rule_state_provider_;
  std::unique_ptr<rules::PhaseProvider> phase_provider_;
  std::unique_ptr<rules::RuleRegistry> rule_registry_;
  std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider_;
  std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider_;
};

}  // namespace api
}  // namespace maliput
