#include "maliput/base/manual_range_value_rule_state_provider.h"

#include <algorithm>
#include <stdexcept>
#include <string>

#include "maliput/base/rule_filter.h"
#include "maliput/common/logger.h"

namespace maliput {

void ManualRangeValueRuleStateProvider::ValidateRuleState(const api::rules::RangeValueRule& range_value_rule,
                                                          const api::rules::RangeValueRule::Range& state) const {
  if (std::find(range_value_rule.states().begin(), range_value_rule.states().end(), state) ==
      range_value_rule.states().end()) {
    MALIPUT_THROW_MESSAGE("Range is not in RangeValueRule " + range_value_rule.id().string() + "'s' states().");
  }
}

void ManualRangeValueRuleStateProvider::SetState(const api::rules::Rule::Id& id,
                                                 const api::rules::RangeValueRule::Range& state,
                                                 const std::optional<api::rules::RangeValueRule::Range>& next_state,
                                                 const std::optional<double>& duration_until) {
  const api::rules::RangeValueRule rule = rulebook_->GetRangeValueRule(id);
  ValidateRuleState(rule, state);
  if (next_state.has_value()) {
    ValidateRuleState(rule, *next_state);
    if (duration_until.has_value()) {
      MALIPUT_THROW_UNLESS(*duration_until > 0.);
    }
  } else {
    MALIPUT_THROW_UNLESS(!duration_until.has_value());
  }

  api::rules::RangeValueRuleStateProvider::StateResult state_result;
  state_result.state = state;
  if (next_state.has_value()) {
    state_result.next = {{*next_state, duration_until}};
  }

  states_[id] = state_result;
}

std::optional<api::rules::RangeValueRuleStateProvider::StateResult> ManualRangeValueRuleStateProvider::DoGetState(
    const api::rules::Rule::Id& id) const {
  const auto it = states_.find(id);
  if (it == states_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<api::rules::RangeValueRuleStateProvider::StateResult> ManualRangeValueRuleStateProvider::DoGetState(
    const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type, double tolerance) const {
  MALIPUT_THROW_UNLESS(tolerance >= 0.);
  const auto query_result_rules = rulebook_->Rules();
  const RangeValueRuleFilter rule_type_filter = [&rule_type](const api::rules::RangeValueRule& rule) {
    return rule.type_id() == rule_type;
  };
  const RangeValueRuleFilter zone_filter = [&road_position, tolerance](const api::rules::RangeValueRule& rule) {
    const api::LaneSRange lane_s_range{road_position.lane->id(),
                                       api::SRange{road_position.pos.s(), road_position.pos.s()}};
    return rule.zone().Intersects(api::LaneSRoute({lane_s_range}), tolerance);
  };
  const auto filtered_rules = FilterRules(query_result_rules, {}, {rule_type_filter, zone_filter});
  if (filtered_rules.range_value_rules.size() > 1) {
    maliput::log()->warn(
        "For rule_type: {} and road_position: [LaneId: {}, LanePos: {}] there are more than one possible rules: ",
        rule_type.string(), road_position.lane->id(), road_position.pos.srh().to_str());
    for (const auto& rule : filtered_rules.range_value_rules) {
      maliput::log()->warn("\tRule id: {} matches with rule_type: {} and road_position: [LaneId: {}, LanePos: {}]",
                           rule.first.string(), rule_type.string(), road_position.lane->id(),
                           road_position.pos.srh().to_str());
    }
  }
  std::optional<api::rules::RangeValueRuleStateProvider::StateResult> current_state{std::nullopt};
  if (!filtered_rules.range_value_rules.empty()) {
    const auto state = states_.find(filtered_rules.range_value_rules.begin()->first);
    MALIPUT_THROW_UNLESS(state != states_.end());
    current_state = std::make_optional<>(state->second);
  }
  return current_state;
}

}  // namespace maliput
