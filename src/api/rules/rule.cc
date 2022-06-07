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
#include "maliput/api/rules/rule.h"

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

namespace {

// Compare whether two map's contents match.
// It does not use std::map::operator==() because the order in the vector is meaningless.
template <class T>
bool CompareMapAttributes(const std::map<std::string, std::vector<T>>& map_a,
                          const std::map<std::string, std::vector<T>>& map_b) {
  if (map_a.size() != map_b.size()) {
    return false;
  }
  for (const auto& key_val : map_a) {
    const auto it = map_b.find(key_val.first);
    if (it == map_b.end()) {
      return false;
    }
    for (const auto& value_id : key_val.second) {
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
    MALIPUT_VALIDATE(!group_id_to_related_rules.first.empty(),
                     "Rule(" + id_.string() + ") contains an empty key in related_rules");
    for (const Rule::Id& rule_id : group_id_to_related_rules.second) {
      MALIPUT_VALIDATE(
          std::count(group_id_to_related_rules.second.begin(), group_id_to_related_rules.second.end(), rule_id) == 1,
          "Rule(" + id_.string() + ") with related_rules that contains a duplicate Rule::Id(" + rule_id.string() +
              ") at key <" + group_id_to_related_rules.first + ">");
    }
  }
}

void Rule::ValidateRelatedUniqueIds(const RelatedUniqueIds& related_unique_ids) const {
  for (const auto& group_id_to_related_unique_ids : related_unique_ids) {
    MALIPUT_VALIDATE(!group_id_to_related_unique_ids.first.empty(),
                     "Rule(" + id_.string() + ") contains an empty key in related_unique_ids");
    for (const UniqueId& unique_id : group_id_to_related_unique_ids.second) {
      MALIPUT_VALIDATE(std::count(group_id_to_related_unique_ids.second.begin(),
                                  group_id_to_related_unique_ids.second.end(), unique_id) == 1,
                       "Rule(" + id_.string() + ") with related_unique_ids that contains a duplicate UniqueId(" +
                           unique_id.string() + ") at key <" + group_id_to_related_unique_ids.first + ">");
    }
  }
}

void Rule::ValidateSeverity(int severity) const {
  MALIPUT_VALIDATE(severity >= 0, "Rule(" + id_.string() + ") has a state whose severity is negative.");
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
