#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

class RangeValueRuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RangeValueRuleStateProvider)

  virtual ~RangeValueRuleStateProvider() = default;

  struct RangeValueResult {
    struct Next {
      RangeValueRule::Range state_range;
      drake::optional<double> duration_until;
    };

    RangeValueRule::Range state_range;

    drake::optional<Next> next;
  };

  drake::optional<RangeValueResult> GetState(const Rule::Id& id) const {
    return DoGetState(id);
  }

 protected:
  RuleStateProvider() = default;

 private:
  virtual drake::optional<DiscreteValueResult> DoGetState(
  		const Rule::Id& id) const = 0;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
