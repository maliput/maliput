#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

namespace {

template <class T>
bool CompareMapAttributes(const std::map<std::string, std::vector<T>>& map_a,
                          const std::map<std::string, std::vector<T>>& map_b) {
  if (map_a.size() != map_b.size()) {
    return false;
  }
  for (const auto& key_val : map_a) {
    const auto it = map_b.find(key_val.first);
    if (it == map_b.end()) {
      false;
    }
    for (const auto value_id : key_val.second) {
      if (std::find(it->second.begin(), it->second.end(), value_id) == it->second.end()) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

bool Rule::State::operator==(const State& other) const {
  return CompareMapAttributes(related_rules, other.related_rules) &&
         CompareMapAttributes(related_unique_ids, other.related_unique_ids) && severity == other.severity;
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

void Rule::ValidateRelatedUniqueIds(const RelatedUniqueIds& related_unique_ids) const {
  for (const auto& group_id_to_related_unique_ids : related_unique_ids) {
    MALIPUT_THROW_UNLESS(!group_id_to_related_unique_ids.first.empty());
    for (const UniqueId& unique_id : group_id_to_related_unique_ids.second) {
      MALIPUT_THROW_UNLESS(std::count(group_id_to_related_unique_ids.second.begin(),
                                      group_id_to_related_unique_ids.second.end(), unique_id) == 1);
    }
  }
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
