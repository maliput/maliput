#include "maliput/base/rule_filter.h"

#include <map>

namespace maliput {
namespace {

/// Filters `map_to_filter` based on `filter_fn`. When `filter_fn` is true, it
/// copies the contents of the key and value of `map_to_filter` into the result.
template <typename K, typename V>
std::map<K, V> FilterMap(const std::map<K, V>& map_to_filter, std::function<bool(const V&)> filter_fn) {
  std::map<K, V> filtered_map;
  for (const auto& k_v : map_to_filter) {
    if (filter_fn(k_v.second)) {
      filtered_map.emplace(k_v.first, k_v.second);
    }
  }
  return filtered_map;
}

}  // namespace

api::rules::RoadRulebook::QueryResults FilterRules(
    const api::rules::RoadRulebook::QueryResults& rules,
    const std::vector<DiscreteValueRuleFilter>& discrete_value_rules_filters,
    const std::vector<RangeValueRuleFilter>& range_value_rules_filters) {
  api::rules::RoadRulebook::QueryResults result(rules);
  for (const auto& filter_fn : discrete_value_rules_filters) {
    result.discrete_value_rules = FilterMap(result.discrete_value_rules, filter_fn);
  }
  for (const auto& filter_fn : range_value_rules_filters) {
    result.range_value_rules = FilterMap(result.range_value_rules, filter_fn);
  }
  return result;
}

}  // namespace maliput
