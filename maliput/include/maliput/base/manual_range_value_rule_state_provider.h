#pragma once

#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/range_value_rule_state_provider.h"

namespace maliput {

/// A trivial implementation of an api::rules::RangeValueRuleStateProvider.
class ManualRangeValueRuleStateProvider : public api::rules::RangeValueRuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManualRangeValueRuleStateProvider)

  ManualRangeValueRuleStateProvider() = default;
  ~ManualRangeValueRuleStateProvider() override = default;

  /// Adds a dynamic state to this provider.
  ///
  /// @throws std::logic_error When a Rule::Id with an ID of @p id already
  ///         exists in this provider.
  void AddState(const api::rules::Rule::Id& id, const api::rules::RangeValueRule::Range& initial_state);

  /// Sets the dynamic state of a RangeValueRule within this provider.
  ///
  /// @throws std::out_of_range if no dynamic state with @p id exists in this
  ///         provider.
  void SetState(const api::rules::Rule::Id& id, const api::rules::RangeValueRule::Range& state);

 private:
  drake::optional<api::rules::RangeValueRuleStateProvider::RangeValueResult> DoGetState(
      const api::rules::Rule::Id& id) const final;

  std::unordered_map<api::rules::Rule::Id, api::rules::RangeValueRule::Range> states_;
};

}  // namespace maliput
