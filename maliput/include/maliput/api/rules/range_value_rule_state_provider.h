#pragma once

#include <optional>

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/state_provider_result.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the state provider of RangeValueRules.
class RangeValueRuleStateProvider {
 public:
  /// The state of a RangeValueRule, returned by
  /// RangeValueRuleStateProvider::GetState(const Rule::Id&).
  using StateResult = StateProviderResult<RangeValueRule::Range>;

  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RangeValueRuleStateProvider)

  virtual ~RangeValueRuleStateProvider() = default;

  /// Gets the state of the RangeValueRule identified by `id`.
  ///
  /// Returns a StateResult struct bearing the state of the rule with the
  /// specified `id`. If `id` is unrecognized, std::nullopt is returned.
  std::optional<StateResult> GetState(const Rule::Id& id) const { return DoGetState(id); }

  /// Gets the state of the RangeValueRule that matches provided `road_position` and `rule_type` values according to
  /// certain `tolerance`.
  ///
  /// Returns a StateResult struct bearing the state of the rule.
  /// If no rule is obtained out of the `road_position` and `rule_type` combination, std::nullopt is returned.
  std::optional<StateResult> GetState(const RoadPosition& road_position, const Rule::TypeId& rule_type,
                                      double tolerance) const {
    return DoGetState(road_position, rule_type, tolerance);
  }

 protected:
  RangeValueRuleStateProvider() = default;

 private:
  virtual std::optional<StateResult> DoGetState(const Rule::Id& id) const = 0;
  virtual std::optional<StateResult> DoGetState(const RoadPosition& road_position, const Rule::TypeId& rule_type,
                                                double tolerance) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
