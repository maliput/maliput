#pragma once

#include <optional>
#include <unordered_map>

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/range_value_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {

/// An implementation of a manual api::rules::RangeValueRuleStateProvider.
/// This enables clients to directly set the states of
/// api::rules::RangeValueRule instances.
class ManualRangeValueRuleStateProvider : public api::rules::RangeValueRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ManualRangeValueRuleStateProvider)

  /// Constructs a ManualRangeValueRuleStateProvider.
  ///
  /// @param rulebook A rulebook pointer to validate Rule::Id and their states.
  ///        It must not be nullptr.
  ///
  /// @throws common::assertion_error When `rulebook` is nullptr.
  explicit ManualRangeValueRuleStateProvider(const api::rules::RoadRulebook* rulebook)
      : api::rules::RangeValueRuleStateProvider(), rulebook_(rulebook) {
    MALIPUT_THROW_UNLESS(rulebook_ != nullptr);
  }

  ~ManualRangeValueRuleStateProvider() override = default;

  /// Sets the state, and optionally the next state and duration until the next
  /// state, of a RangeValueRule.
  ///
  /// If @p id is new, a new entry is added to this provider.
  ///
  /// @p next_state and @p duration_until are not managed by this
  /// provider, i.e. it is the client's responsibility to govern transitions from
  /// @p state to @p next_state after @p duration_until (if provided) time.
  ///
  /// @param id RangeValueRule's ID. It must be recognized by the `rulebook`
  ///        specified at time of construction.
  /// @param state The state of the rule. It must be one in
  ///        RangeValueRule::ranges().
  /// @param next_state The optional next state of the rule. It must be one in
  ///        RangeValueRule::ranges().
  /// @param duration_until If known, the estimated time until the transition to
  ///        the next state, relative to when this method is called. When
  ///        provided, it must be positive and @p next_state must not be nullopt.
  ///
  /// @throws std::out_of_range When @p id is unrecognized by the `rulebook`
  ///         specified at time of construction.
  /// @throws common::assertion_error When @p state does not match any state
  ///         in RangeValueRule::ranges().
  /// @throws common::assertion_error When @p next_state does not match any state
  ///         in RangeValueRule::ranges().
  /// @throws common::assertion_error When @p duration_until is not positive or
  ///         it is provided when @p next_state is nullopt.
  void SetState(const api::rules::Rule::Id& id, const api::rules::RangeValueRule::Range& state,
                const std::optional<api::rules::RangeValueRule::Range>& next_state,
                const std::optional<double>& duration_until);

 private:
  // This function has been marked as virtual because other providers might
  // benefit from injecting their own getter and then forwarding calls to this
  // function.
  virtual std::optional<api::rules::RangeValueRuleStateProvider::StateResult> DoGetState(
      const api::rules::Rule::Id& id) const override;

  // This function has been marked as virtual because other providers might
  // benefit from injecting their own getter and then forwarding calls to this
  // function.
  virtual std::optional<api::rules::RangeValueRuleStateProvider::StateResult> DoGetState(
      const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type,
      double tolerance) const override;

  // @throws common::assertion_error When @p state is unrecognized in
  //         @p range_value_rule's ranges.
  void ValidateRuleState(const api::rules::RangeValueRule& range_value_rule,
                         const api::rules::RangeValueRule::Range& state) const;

  std::unordered_map<api::rules::Rule::Id, api::rules::RangeValueRuleStateProvider::StateResult> states_;
  const api::rules::RoadRulebook* rulebook_{nullptr};
};

}  // namespace maliput
