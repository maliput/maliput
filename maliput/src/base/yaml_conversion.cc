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

bool IsDiscreteValue(const YAML::Node& rule_node) {
  if (!rule_node[kValue].IsDefined()) {
    return false;
  }
  int attribute_count{1};
  if (rule_node[kSeverity].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[kRelatedRules].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[kRelatedUniqueIds].IsDefined()) {
    attribute_count++;
  }
  return rule_node.size() == attribute_count;
}

bool IsRangeValue(const YAML::Node& rule_node) {
  if (!rule_node[kRange].IsDefined() || !rule_node[kDescription].IsDefined()) {
    return false;
  }
  int attribute_count{2};
  if (rule_node[kSeverity].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[kRelatedRules].IsDefined()) {
    attribute_count++;
  }
  if (rule_node[kRelatedUniqueIds].IsDefined()) {
    attribute_count++;
  }
  return rule_node.size() == attribute_count;
}

int GetSeverityFromYamlNode(const YAML::Node& node) {
  if (node[kSeverity].IsDefined()) {
    const int severity = node[kSeverity].as<int>();
    MALIPUT_THROW_UNLESS(severity >= 0);
    return severity;
  }
  return api::rules::Rule::State::kStrict;
}

std::pair<double, double> GetRangeMinMaxValuesFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[kRange].IsSequence());
  MALIPUT_THROW_UNLESS(node[kRange].size() == 2);
  const double min = node[kRange][0].as<double>();
  const double max = node[kRange][1].as<double>();
  MALIPUT_THROW_UNLESS(min <= max);
  return std::make_pair(min, max);
}

std::string GetValueFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[kValue].IsDefined());
  return node[kValue].as<std::string>();
}

std::string GetDescriptionFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[kDescription].IsDefined());
  return node[kDescription].as<std::string>();
}

}  // namespace maliput
