#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

class DiscreteValueRuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValueRuleStateProvider)

  virtual ~DiscreteValueRuleStateProvider() = default;

  struct DiscreteValueResult {
    struct Next {
      std::string state_value;
      drake::optional<double> duration_until;
    };

    std::string state_value;

    drake::optional<Next> next;
  };

  drake::optional<DiscreteValueResult> GetState(const Rule::Id& id) const {
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
