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
#include "maliput/base/yaml_conversion.h"

#include "maliput/api/rules/rule.h"

namespace YAML {

Node convert<maliput::api::SRange>::encode(const maliput::api::SRange& rhs) {
  Node node;
  node.push_back(rhs.s0());
  node.push_back(rhs.s1());
  return node;
}

bool convert<maliput::api::SRange>::decode(const Node& node, maliput::api::SRange& rhs) {
  if (!node.IsSequence() || node.size() != 2) {
    return false;
  }
  rhs.set_s0(node[0].as<double>());
  rhs.set_s1(node[1].as<double>());
  return true;
}

}  // namespace YAML

namespace maliput {

const char* DiscreteValueRuleConstants::kValue{"value"};
const char* DiscreteValueRuleConstants::kValues{"values"};
const char* RangeValueRuleConstants::kDescription{"description"};
const char* RangeValueRuleConstants::kRange{"range"};
const char* RangeValueRuleConstants::kRanges{"ranges"};
const char* RuleConstants::kId{"id"};
const char* RuleConstants::kLaneId{"lane_id"};
const char* RuleConstants::kRelatedRules{"related_rules"};
const char* RuleConstants::kRelatedUniqueIds{"related_unique_ids"};
const char* RuleConstants::kSeverity{"severity"};
const char* RuleConstants::kSRange{"s_range"};
const char* RuleConstants::kType{"type"};
const char* RuleConstants::kZone{"zone"};

bool IsDiscreteValue(const YAML::Node& rule_node) {
  if (!rule_node[DiscreteValueRuleConstants::kValue].IsDefined()) {
    return false;
  }
  int attribute_count{1};
  if (rule_node[RuleConstants::kSeverity].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[RuleConstants::kRelatedRules].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[RuleConstants::kRelatedUniqueIds].IsDefined()) {
    attribute_count++;
  }
  return static_cast<int>(rule_node.size()) == attribute_count;
}

bool IsRangeValue(const YAML::Node& rule_node) {
  if (!rule_node[RangeValueRuleConstants::kRange].IsDefined() ||
      !rule_node[RangeValueRuleConstants::kDescription].IsDefined()) {
    return false;
  }
  int attribute_count{2};
  if (rule_node[RuleConstants::kSeverity].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[RuleConstants::kRelatedRules].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[RuleConstants::kRelatedUniqueIds].IsDefined()) {
    attribute_count++;
  }
  return static_cast<int>(rule_node.size()) == attribute_count;
}

int GetSeverityFromYamlNode(const YAML::Node& node) {
  if (node[RuleConstants::kSeverity].IsDefined()) {
    const int severity = node[RuleConstants::kSeverity].as<int>();
    MALIPUT_THROW_UNLESS(severity >= 0);
    return severity;
  }
  return api::rules::Rule::State::kStrict;
}

std::pair<double, double> GetRangeMinMaxValuesFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[RangeValueRuleConstants::kRange].IsSequence());
  MALIPUT_THROW_UNLESS(node[RangeValueRuleConstants::kRange].size() == 2);
  const double min = node[RangeValueRuleConstants::kRange][0].as<double>();
  const double max = node[RangeValueRuleConstants::kRange][1].as<double>();
  MALIPUT_THROW_UNLESS(min <= max);
  return std::make_pair(min, max);
}

std::string GetValueFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[DiscreteValueRuleConstants::kValue].IsDefined());
  return node[DiscreteValueRuleConstants::kValue].as<std::string>();
}

std::string GetDescriptionFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[RangeValueRuleConstants::kDescription].IsDefined());
  return node[RangeValueRuleConstants::kDescription].as<std::string>();
}

}  // namespace maliput
