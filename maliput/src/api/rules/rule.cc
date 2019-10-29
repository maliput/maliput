#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

bool Rule::State::operator==(const State& other) const {
  if (related_rules.size() != other.related_rules.size()) {
    return false;
  }
  for (const auto& key_val : related_rules) {
    const auto it = other.related_rules.find(key_val.first);
    if (it == other.related_rules.end()) {
      false;
    }
    for (const Rule::Id& rule_id : key_val.second) {
      if (std::find(it->second.begin(), it->second.end(), rule_id) == it->second.end()) {
        return false;
      }
    }
  }
  if (related_unique_ids.size() != other.related_unique_ids.size()) {
    return false;
  }
  for (const auto& key_val : related_unique_ids) {
    const auto it = other.related_unique_ids.find(key_val.first);
    if (it == other.related_unique_ids.end()) {
      false;
    }
    for (const UniqueId& unique_id : key_val.second) {
      if (std::find(it->second.begin(), it->second.end(), unique_id) == it->second.end()) {
        return false;
      }
    }
  }
  return severity == other.severity;
}

void Rule::ValidateRelatedRules(const Rule::RelatedRules& related_rules) const {
  for (const auto& group_id_to_related_rules : related_rules) {
    MALIPUT_THROW_UNLESS(!group_id_to_related_rules.first.empty());
    for (const Rule::Id& rule_id : group_id_to_related_rules.second) {
      MALIPUT_THROW_UNLESS(
          std::count(group_id_to_related_rules.second.begin(), group_id_to_related_rules.second.end(), rule_id) == 1);
    }
  }
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
