#pragma once

#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/range_value_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {

/// An implementation of a manual api::rules::RangeValueRuleStateProvider.
/// This enables clients to directly set the states of
/// api::rules::RangeValueRule instances.
class ManualRangeValueRuleStateProvider : public api::rules::RangeValueRuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManualRangeValueRuleStateProvider)

  /// Constructs a ManualRangeValueRuleStateProvider.
  ///
  /// @param rulebook A rulebook pointer to validate Rule::Id and their states.
  ///        It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When `rulebook` is nullptr.
  explicit ManualRangeValueRuleStateProvider(const api::rules::RoadRulebook* rulebook)
      : api::rules::RangeValueRuleStateProvider(), rulebook_(rulebook) {
    MALIPUT_THROW_UNLESS(rulebook_ != nullptr);
  }

  ~ManualRangeValueRuleStateProvider() override = default;

  /// Registers a RangeValueRule to this provider.
  ///
  /// @throws std::logic_error When @p id already registered within this
  ///         provider.
  /// @throws std::out_of_range When @p id is unrecognized by the `rulebook`
  ///         specified at time of construction.
  /// @throws maliput::common::assertion_error When @p initial_state does not
  ///         match any state in RangeValueRule::ranges().
  void Register(const api::rules::Rule::Id& id, const api::rules::RangeValueRule::Range& initial_state);

  /// Sets the state, and optionally the next state, of a RangeValueRule
  /// within this provider.
  ///
  /// @throws std::out_of_range When @p id is not registered within this
  ///         provider.
  /// @throws std::out_of_range When @p id is unrecognized by `rulebook`.
  /// @throws maliput::common::assertion_error When @p initial_state does not
  ///         match any state in RangeValueRule::ranges().
  /// @throws maliput::common::assertion_error When @p next_state.range_state does
  ///         not match any state in RangeValueRule::ranges().
  void SetState(const api::rules::Rule::Id& id, const api::rules::RangeValueRule::Range& state,
                const drake::optional<api::rules::RangeValueRule::Range>& next_state);

 private:
  drake::optional<api::rules::RangeValueRuleStateProvider::StateResult> DoGetState(
      const api::rules::Rule::Id& id) const final;

  // @throws maliput::common::assertion_error When @p state is unrecognized in
  //         @p range_value_rule's ranges.
  void ValidateRuleState(const api::rules::RangeValueRule& range_value_rule,
                         const api::rules::RangeValueRule::Range& state) const;

  std::unordered_map<api::rules::Rule::Id, api::rules::RangeValueRuleStateProvider::StateResult> states_;
  const api::rules::RoadRulebook* rulebook_{nullptr};
};

}  // namespace maliput
