#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/regions.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/speed_limit_rule.h"

namespace maliput {

/// SimpleRulebook is a simple concrete implementation of the
/// api::rules::RoadRulebook abstract interface.
class SimpleRulebook : public api::rules::RoadRulebook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleRulebook);

  /// Constructs an empty SimpleRulebook (i.e., containing no rules).
  SimpleRulebook();

  ~SimpleRulebook() override;

  /// Removes all rules.
  void RemoveAll();

  /// Adds a new RightOfWayRule.
  ///
  /// @throws maliput::common::assertion_error if a rule with the same ID
  /// already exists in the SimpleRulebook.
  void AddRule(const api::rules::RightOfWayRule& rule);

  /// Removes the RightOfWayRule labeled by `id`.
  ///
  /// @throws maliput::common::assertion_error if no such rule exists.
  void RemoveRule(const api::rules::RightOfWayRule::Id& id);

  /// Adds a new SpeedLimitRule.
  ///
  /// @throws maliput::common::assertion_error if a rule with the same ID
  /// already exists in the SimpleRulebook.
  void AddRule(const api::rules::SpeedLimitRule& rule);

  /// Removes the SpeedLimitRule labeled by `id`.
  ///
  /// @throws maliput::common::assertion_error if no such rule exists.
  void RemoveRule(const api::rules::SpeedLimitRule::Id& id);

  /// Adds a new DirectionUsageRule.
  ///
  /// @throws maliput::common::assertion_error if a rule with the same ID
  /// already exists in the SimpleRulebook.
  void AddRule(const api::rules::DirectionUsageRule& rule);

  /// Removes the DirectionUsageRule labeled by `id`.
  ///
  /// @throws maliput::common::assertion_error if no such rule exists.
  void RemoveRule(const api::rules::DirectionUsageRule::Id& id);

 private:
  api::rules::RoadRulebook::QueryResults DoFindRules(const std::vector<api::LaneSRange>& ranges,
                                                     double tolerance) const override;
  api::rules::RightOfWayRule DoGetRule(const api::rules::RightOfWayRule::Id& id) const override;
  api::rules::SpeedLimitRule DoGetRule(const api::rules::SpeedLimitRule::Id& id) const override;
  api::rules::DirectionUsageRule DoGetRule(const api::rules::DirectionUsageRule::Id& id) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
