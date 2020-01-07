#include "maliput/base/rule_registry_loader.h"

#include "yaml-cpp/yaml.h"

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"

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

// Constants to identify attributes of DiscreteValueRule Types and DiscreteValueRule Types.
constexpr const char* const kValue = "value";
constexpr const char* const kRange = "range";
constexpr const char* const kDescription = "description";
constexpr const char* const kSeverity = "severity";
constexpr const char* const kRelatedRules = "related_rules";
constexpr const char* const kRelatedUniqueIds = "related_unique_ids";
// Label to identify rule type.
enum class RuleType {
  // Label for a DiscreteValueRule Type.
  kDiscreteValueRuleType,
  // Label for a RangeValueRule Type.
  kRangeValueRuleType,
  // Label for an unidentified rule type.
  kUnknownRuleType,
};

// Determines whether the `rule_node` corresponds to a api::rules::DiscreteValueRule::DiscreteValue description.
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

// Determines whether the `rule_node` corresponds to a api::rules::RangeValueRule::Range description.
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

// Returns the severity field value from the `node`.
int GetSeverityFromYamlNode(const YAML::Node& node) {
  if (node[kSeverity].IsDefined()) {
    const int severity = node["severity"].as<int>();
    MALIPUT_THROW_UNLESS(severity >= 0);
    return severity;
  }
  return Rule::State::kStrict;
}

// Returns a Rule::RelatedRules containing the related_rules field value from the `node`.
Rule::RelatedRules GetRelatedRuleFromYamlNode(const YAML::Node& node) {
  return node[kRelatedRules].IsDefined() ? node[kRelatedRules].as<Rule::RelatedRules>() : Rule::RelatedRules{};
}

// Returns a Rule::RelatedUniqueIds containing the related_unique_ids field value from the `node`.
Rule::RelatedUniqueIds GetRelatedUniqueIdsFromYamlNode(const YAML::Node& node) {
  return node[kRelatedUniqueIds].IsDefined() ? node[kRelatedUniqueIds].as<Rule::RelatedUniqueIds>()
                                             : Rule::RelatedUniqueIds{};
}

// Returns a std::pair containing the min and max of the range field values from the node.
std::pair<double, double> GetRangeMinMaxValuesFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[kRange].size() == 2);
  double min = node[kRange][0].as<double>();
  double max = node[kRange][1].as<double>();
  MALIPUT_THROW_UNLESS(min <= max);
  return std::make_pair(min, max);
}

// Register a DiscreteValueRule type within the `rule_registry`.
void RegisterDiscreteValueRuleType(RuleRegistry* rule_registry, const std::string& rule_type_id,
                                   const YAML::Node& rule_nodes) {
  MALIPUT_THROW_UNLESS(rule_registry != nullptr);

  std::vector<DiscreteValueRule::DiscreteValue> discrete_values{};
  for (const YAML::Node& rule_node : rule_nodes) {
    discrete_values.push_back(DiscreteValueRule::DiscreteValue{
        GetSeverityFromYamlNode(rule_node), GetRelatedRuleFromYamlNode(rule_node),
        GetRelatedUniqueIdsFromYamlNode(rule_node), rule_node[kValue].as<std::string>()});
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
                                           rule_node[kDescription].as<std::string>(), min_max.first, min_max.second});
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
