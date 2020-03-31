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
