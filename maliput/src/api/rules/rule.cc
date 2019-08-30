#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

void Rule::ValidateRelatedRules(const Rule::RelatedRules& related_rules) const {
  for (const auto& group_id_to_related_rules : related_rules) {
    MALIPUT_THROW_UNLESS(!group_id_to_related_rules.first.empty());
    MALIPUT_THROW_UNLESS(!group_id_to_related_rules.second.empty());
    for (const Rule::Id& rule_id : group_id_to_related_rules.second) {
      MALIPUT_THROW_UNLESS(
          std::count(group_id_to_related_rules.second.begin(), group_id_to_related_rules.second.end(), rule_id) == 1);
    }
  }
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
