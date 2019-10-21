// Copyright 2019 Toyota Research Institute
#include "maliput/api/rules/rule_types.h"

#include <string>

namespace maliput {
namespace api {
namespace rules {

maliput::api::rules::Rule::TypeId DirectionUsageRuleTypeId() {
  return maliput::api::rules::Rule::TypeId("DirectionUsageRuleType");
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
