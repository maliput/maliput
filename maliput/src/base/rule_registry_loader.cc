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
#include "maliput/base/rule_registry_loader.h"

#include <yaml-cpp/yaml.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/base/yaml_conversion.h"

using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;
using maliput::api::rules::RuleRegistry;

namespace YAML {

// This struct is used for encoding and decoding Rule::RelatedRules and Rule::RelatedUniqueIds with a YAML::Node.
template <typename T>
struct convert<std::map<std::string, std::vector<T>>> {
  static Node encode(const std::map<std::string, std::vector<T>>& rhs) {
    Node node;
    for (const auto& key_values : rhs) {
      node.push_back(key_values.first);
    }
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, std::map<std::string, std::vector<T>>& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    for (const auto& value : node) {
      rhs[value.as<std::string>()];
    }
    return true;
  }
};

}  // namespace YAML

namespace maliput {
namespace {

// Returns wether the `rule_nodes` refer to a RuleType::kDiscreteValueRuleType or
// RuleType::kRangeValueRuleType. RuleType::kUnknownRuleType is returned if the rule type is not well-defined.
RuleType EvaluateRuleType(const YAML::Node& rule_nodes) {
  MALIPUT_THROW_UNLESS(rule_nodes.IsSequence());
  MALIPUT_THROW_UNLESS(rule_nodes.size());
  RuleType rule_type{RuleType::kUnknownRuleType};
  for (const auto& rule_node : rule_nodes) {
    MALIPUT_THROW_UNLESS(rule_node.IsMap());
    if (IsDiscreteValue(rule_node)) {
      if (rule_type == RuleType::kRangeValueRuleType) {
        rule_type = RuleType::kUnknownRuleType;
        break;
      }
      rule_type = RuleType::kDiscreteValueRuleType;
    } else if (IsRangeValue(rule_node)) {
      if (rule_type == RuleType::kDiscreteValueRuleType) {
        rule_type = RuleType::kUnknownRuleType;
        break;
      }
      rule_type = RuleType::kRangeValueRuleType;
    }
  }
  return rule_type;
}

// Returns a Rule::RelatedRules containing the related_rules field value from the `node`.
Rule::RelatedRules GetRelatedRuleFromYamlNode(const YAML::Node& node) {
  return node[RuleConstants::kRelatedRules].IsDefined() ? node[RuleConstants::kRelatedRules].as<Rule::RelatedRules>()
                                                        : Rule::RelatedRules{};
}

// Returns a Rule::RelatedUniqueIds containing the related_unique_ids field value from the `node`.
Rule::RelatedUniqueIds GetRelatedUniqueIdsFromYamlNode(const YAML::Node& node) {
  return node[RuleConstants::kRelatedUniqueIds].IsDefined()
             ? node[RuleConstants::kRelatedUniqueIds].as<Rule::RelatedUniqueIds>()
             : Rule::RelatedUniqueIds{};
}

// Register a DiscreteValueRule type within the `rule_registry`.
void RegisterDiscreteValueRuleType(RuleRegistry* rule_registry, const std::string& rule_type_id,
                                   const YAML::Node& rule_nodes) {
  MALIPUT_THROW_UNLESS(rule_registry != nullptr);

  std::vector<DiscreteValueRule::DiscreteValue> discrete_values{};
  for (const YAML::Node& rule_node : rule_nodes) {
    discrete_values.push_back(DiscreteValueRule::DiscreteValue{
        GetSeverityFromYamlNode(rule_node), GetRelatedRuleFromYamlNode(rule_node),
        GetRelatedUniqueIdsFromYamlNode(rule_node), rule_node[DiscreteValueRuleConstants::kValue].as<std::string>()});
  }
  rule_registry->RegisterDiscreteValueRule(Rule::TypeId(rule_type_id), discrete_values);
}

// Register a RangeValueRule type within the `rule_registry`.
void RegisterRangeValueRuleType(RuleRegistry* rule_registry, const std::string& rule_type_id,
                                const YAML::Node& rule_nodes) {
  MALIPUT_THROW_UNLESS(rule_registry != nullptr);

  std::vector<RangeValueRule::Range> ranges{};
  for (const YAML::Node& rule_node : rule_nodes) {
    const std::pair<double, double> min_max{GetRangeMinMaxValuesFromYamlNode(rule_node)};
    ranges.push_back(RangeValueRule::Range{GetSeverityFromYamlNode(rule_node), GetRelatedRuleFromYamlNode(rule_node),
                                           GetRelatedUniqueIdsFromYamlNode(rule_node),
                                           GetDescriptionFromYamlNode(rule_node), min_max.first, min_max.second});
  }
  rule_registry->RegisterRangeValueRule(Rule::TypeId(rule_type_id), ranges);
}

// Returns a api::rules::RuleRegistry created from `root_node`.
std::unique_ptr<api::rules::RuleRegistry> BuildFrom(const YAML::Node& root_node) {
  MALIPUT_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& rule_registry_node = root_node["RuleRegistry"];
  MALIPUT_THROW_UNLESS(rule_registry_node.IsDefined());
  MALIPUT_THROW_UNLESS(rule_registry_node.IsMap());

  std::unique_ptr<api::rules::RuleRegistry> rule_registry = std::make_unique<api::rules::RuleRegistry>();

  for (YAML::const_iterator rule_type_id_it = rule_registry_node.begin(); rule_type_id_it != rule_registry_node.end();
       ++rule_type_id_it) {
    const RuleType rule_type = EvaluateRuleType(rule_type_id_it->second);
    MALIPUT_THROW_UNLESS(rule_type != RuleType::kUnknownRuleType);
    switch (rule_type) {
      case RuleType::kDiscreteValueRuleType:
        RegisterDiscreteValueRuleType(rule_registry.get(), rule_type_id_it->first.as<std::string>(),
                                      rule_type_id_it->second);
        break;
      case RuleType::kRangeValueRuleType:
        RegisterRangeValueRuleType(rule_registry.get(), rule_type_id_it->first.as<std::string>(),
                                   rule_type_id_it->second);
        break;
      case RuleType::kUnknownRuleType:
        MALIPUT_THROW_MESSAGE("Unknown Rule Type.");
        break;
    }
  }
  return rule_registry;
}

}  // namespace

std::unique_ptr<api::rules::RuleRegistry> LoadRuleRegistry(const std::string& input) {
  return BuildFrom(YAML::Load(input));
}

std::unique_ptr<api::rules::RuleRegistry> LoadRuleRegistryFromFile(const std::string& filename) {
  return BuildFrom(YAML::LoadFile(filename));
}

}  // namespace maliput
