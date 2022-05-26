// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
