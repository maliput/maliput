#include "maliput/base/road_rulebook_loader.h"

#include <sstream>
#include <stdexcept>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/rule_registry.h"
#include "maliput/common/logger.h"
#include "maliput/common/maliput_throw.h"

using maliput::api::Lane;
using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::SRange;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;
using maliput::api::rules::RuleRegistry;

namespace YAML {

// This struct is used for encoding and decoding api::SRange with a YAML::Node.
template <>
struct convert<SRange> {
  static Node encode(const SRange& rhs) {
    Node node;
    node.push_back(rhs.s0());
    node.push_back(rhs.s1());
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, SRange& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    rhs.set_s0(node[0].as<double>());
    rhs.set_s1(node[1].as<double>());
    return true;
  }
};

// This struct is used for encoding and decoding Rule::RelatedRules and Rule::RelatedUniqueIds with a YAML::Node.
template <typename T>
struct convert<std::map<std::string, std::vector<T>>> {
  static Node encode(const std::map<std::string, std::vector<T>>& rhs) {
    Node node;
    for (const auto& key_values : rhs) {
      for (const auto& values : key_values.second) {
        node[key_values.first].push_back(values);
      }
    }
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, std::map<std::string, std::vector<T>>& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    for (const auto& key_values : node) {
      for (const auto& rule : key_values.first) {
        rhs[key_values.first.as<std::string>()].push_back(T{rule.as<std::string>()});
      }
    }
    return true;
  }
};

}  // namespace YAML

namespace maliput {
namespace {

// Constants to identify attributes of DiscreteValueRule Types and DiscreteValueRule Types.
constexpr const char* const kDescription = "description";
constexpr const char* const kId = "id";
constexpr const char* const kLaneId = "lane_id";
constexpr const char* const kRange = "range";
constexpr const char* const kRanges = "ranges";
constexpr const char* const kRelatedRules = "related_rules";
constexpr const char* const kRelatedUniqueIds = "related_unique_ids";
constexpr const char* const kSeverity = "severity";
constexpr const char* const kSRange = "s_range";
constexpr const char* const kType = "type";
constexpr const char* const kValue = "value";
constexpr const char* const kValues = "values";
constexpr const char* const kZone = "zone";
// Label to identify rule type.
enum class RuleType {
  // Label for a DiscreteValueRule Type.
  kDiscreteValueRuleType,
  // Label for a RangeValueRule Type.
  kRangeValueRuleType,
  // Label for an unidentified rule type.
  kUnknownRuleType,
};

// Determines whether the `rule_node` corresponds to a api::rules::DiscreteValueRule description.
bool IsDiscreteValueRule(const YAML::Node& rule_node) {
  if (!rule_node[kId].IsDefined() || !rule_node[kType].IsDefined() || !rule_node[kZone].IsDefined() ||
      !rule_node[kValues].IsDefined()) {
    return false;
  }
  const YAML::Node& values_node = rule_node[kValues];
  bool result = true;
  for (const auto& discrete_value_node : values_node) {
    if (!rule_node[kValue].IsDefined()) {
      result = false;
      break;
    }
    int attribute_count{1};
    if (values_node[kSeverity].IsDefined()) {
      attribute_count++;
    }
    if (values_node[kRelatedRules].IsDefined()) {
      attribute_count++;
    }
    if (values_node[kRelatedUniqueIds].IsDefined()) {
      attribute_count++;
    }
    if (values_node.size() != attribute_count) {
      result = false;
      break;
    }
  }
  return result;
}

// Determines whether the `rule_node` corresponds to a api::rules::RangeValueRule description.
bool IsRangeValueRule(const YAML::Node& rule_node) {
  if (!rule_node[kId].IsDefined() || !rule_node[kType].IsDefined() || !rule_node[kZone].IsDefined() ||
      !rule_node[kRanges].IsDefined()) {
    return false;
  }
  const YAML::Node& ranges_node = rule_node[kRanges];
  bool result = true;
  for (const auto& range_value_node : ranges_node) {
    if (!rule_node[kRange].IsDefined() || rule_node[kDescription].IsDefined()) {
      result = false;
      break;
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
    if (ranges_node.size() != attribute_count) {
      result = false;
      break;
    }
  }
  return result;
}

// Returns whether the `rule_nodes` refer to a RuleType::kDiscreteValueRuleType or
// RuleType::kRangeValueRuleType. RuleType::kUnknownRuleType is returned if the rule type is not well-defined.
RuleType EvaluateRuleType(const YAML::Node& rule_nodes) {
  MALIPUT_THROW_UNLESS(rule_nodes.IsSequence());
  MALIPUT_THROW_UNLESS(rule_nodes.size());
  RuleType rule_type{RuleType::kUnknownRuleType};
  for (const auto& rule_node : rule_nodes) {
    MALIPUT_THROW_UNLESS(rule_node.IsMap());
    if (IsDiscreteValueRule(rule_node)) {
      if (rule_type == RuleType::kRangeValueRuleType) {
        rule_type = RuleType::kUnknownRuleType;
        break;
      }
      rule_type = RuleType::kDiscreteValueRuleType;
    } else if (IsRangeValueRule(rule_node)) {
      if (rule_type == RuleType::kDiscreteValueRuleType) {
        rule_type = RuleType::kUnknownRuleType;
        break;
      }
      rule_type = RuleType::kRangeValueRuleType;
    }
  }
  return rule_type;
}

// Returns a Rule::Id containing the id field value from the `node`.
Rule::Id GetRuleIdFromYamlNode(const YAML::Node& rule_node) {
  MALIPUT_THROW_UNLESS(rule_node[kId].IsDefined());
  return Rule::Id{rule_node[kId].as<std::string>()};
}

// Returns a Rule::TypeId containing the type field value from the `node`.
Rule::TypeId GetRuleTypeIdFromYamlNode(const YAML::Node& rule_node) {
  MALIPUT_THROW_UNLESS(rule_node[kType].IsDefined());
  return Rule::TypeId{rule_node[kType].as<std::string>()};
}

SRange GetSRange(const Lane* lane, const YAML::Node& lane_node) {
  if (lane_node[kSRange]) {
    const SRange s_range = lane_node[kSRange].as<SRange>();
    MALIPUT_THROW_UNLESS(s_range.s0() >= 0);
    MALIPUT_THROW_UNLESS(s_range.s1() <= lane->length());
    return s_range;
  } else {
    return SRange(0, lane->length());
  }
}

LaneSRange GetLaneSRangeFromYamlNode(const YAML::Node& lane_s_range_node, const api::RoadGeometry* road_geometry) {
  MALIPUT_THROW_UNLESS(lane_s_range_node.IsMap());
  MALIPUT_THROW_UNLESS(lane_s_range_node[kLaneId].IsDefined());
  const LaneId lane_id{lane_s_range_node[kLaneId].as<std::string>()};
  const Lane* lane = road_geometry->ById().GetLane(lane_id);
  MALIPUT_THROW_UNLESS(lane != nullptr);
  const SRange s_range = GetSRange(lane, lane_s_range_node);
  return LaneSRange(lane_id, s_range);
}

// Returns a api::LaneSRoute containing the zone field value from the `node`.
LaneSRoute GetZoneFromYamlNode(const YAML::Node& rule_node, const api::RoadGeometry* road_geometry) {
  MALIPUT_THROW_UNLESS(rule_node[kZone].IsDefined());
  MALIPUT_THROW_UNLESS(rule_node[kZone].IsSequence());
  std::vector<LaneSRange> zone;
  for (const auto& lane_s_range_node : rule_node[kZone]) {
    zone.push_back(GetLaneSRangeFromYamlNode(lane_s_range_node, road_geometry));
  }
  return LaneSRoute{zone};
}

// Returns the severity field value from the `node`.
int GetSeverityFromYamlNode(const YAML::Node& node) {
  if (node[kSeverity].IsDefined()) {
    const int severity = node[kSeverity].as<int>();
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

// Returns a std::string containing the value field value from the `node`.
std::string GetValueFromYamlNode(const YAML::Node& node) { return node[kValue].as<std::string>(); }

// Returns a std::pair containing the min and max of the range field values from the node.
std::pair<double, double> GetRangeMinMaxValuesFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[kRange].size() == 2);
  double min = node[kRange][0].as<double>();
  double max = node[kRange][1].as<double>();
  MALIPUT_THROW_UNLESS(min <= max);
  return std::make_pair(min, max);
}

// Returns a std::string containing the description field value from the `node`.
std::string GetDescriptionFromYamlNode(const YAML::Node& node) { return node[kDescription].as<std::string>(); }

// Add a DiscreteValueRule within the `rulebook`.
void AddDiscreteValueRule(ManualRulebook* rulebook, const YAML::Node& rule_node, const api::RoadGeometry* road_geometry,
                          const RuleRegistry& rule_registry) {
  MALIPUT_THROW_UNLESS(rulebook != nullptr);
  MALIPUT_THROW_UNLESS(rule_node.IsMap());
  MALIPUT_THROW_UNLESS(rule_node[kValues].IsDefined());
  std::vector<DiscreteValueRule::DiscreteValue> discrete_values;
  for (const auto& discrete_value_node : rule_node[kValues]) {
    discrete_values.push_back(MakeDiscreteValue(
        GetSeverityFromYamlNode(discrete_value_node), GetRelatedRuleFromYamlNode(discrete_value_node),
        GetRelatedUniqueIdsFromYamlNode(discrete_value_node), GetValueFromYamlNode(discrete_value_node)));
  }
  DiscreteValueRule discrete_value_rule{
      rule_registry.BuildDiscreteValueRule(GetRuleIdFromYamlNode(rule_node), GetRuleTypeIdFromYamlNode(rule_node),
                                           GetZoneFromYamlNode(rule_node, road_geometry), discrete_values)};
  rulebook->AddRule(discrete_value_rule);
}

// Add a RangeValueRule within the `rulebook`.
void AddRangeValueRule(ManualRulebook* rulebook, const YAML::Node& rule_node, const api::RoadGeometry* road_geometry,
                       const RuleRegistry& rule_registry) {
  MALIPUT_THROW_UNLESS(rulebook != nullptr);
  MALIPUT_THROW_UNLESS(rule_node.IsMap());
  MALIPUT_THROW_UNLESS(rule_node[kRanges].IsDefined());
  std::vector<RangeValueRule::Range> range_values;
  for (const auto& range_value_node : rule_node[kRanges]) {
    const double min = GetRangeMinMaxValuesFromYamlNode(range_value_node).first;
    const double max = GetRangeMinMaxValuesFromYamlNode(range_value_node).second;
    range_values.push_back(MakeRange(
        GetSeverityFromYamlNode(range_value_node), GetRelatedRuleFromYamlNode(range_value_node),
        GetRelatedUniqueIdsFromYamlNode(range_value_node), GetDescriptionFromYamlNode(range_value_node), min, max));
  }

  RangeValueRule range_value_rule{
      rule_registry.BuildRangeValueRule(GetRuleIdFromYamlNode(rule_node), GetRuleTypeIdFromYamlNode(rule_node),
                                        GetZoneFromYamlNode(rule_node, road_geometry), range_values)};
  rulebook->AddRule(range_value_rule);
}

std::unique_ptr<api::rules::RoadRulebook> BuildFrom(const api::RoadGeometry* road_geometry, const YAML::Node& root_node,
                                                    const RuleRegistry& rule_registry) {
  MALIPUT_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& rulebook_node = root_node["RoadRulebook"];
  MALIPUT_THROW_UNLESS(rulebook_node.IsDefined());
  MALIPUT_THROW_UNLESS(rulebook_node.IsMap());
  std::unique_ptr<ManualRulebook> rulebook = std::make_unique<ManualRulebook>();
  for (const YAML::Node& rule_node : rulebook_node) {  //
    MALIPUT_THROW_UNLESS(rule_node.IsSequence());
    const RuleType rule_type = EvaluateRuleType(rule_node);
    switch (rule_type) {
      case RuleType::kDiscreteValueRuleType:
        AddDiscreteValueRule(rulebook.get(), rule_node, road_geometry, rule_registry);
        break;
      case RuleType::kRangeValueRuleType:
        AddRangeValueRule(rulebook.get(), rule_node, road_geometry, rule_registry);
        break;
      case RuleType::kUnknownRuleType:
        MALIPUT_THROW_MESSAGE("Unknown Rule Type.");
        break;
    }
  }
  return rulebook;
}

}  // namespace

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebook(const api::RoadGeometry* road_geometry,
                                                           const std::string& input,
                                                           const RuleRegistry& rule_registry) {
  return BuildFrom(road_geometry, YAML::Load(input), rule_registry);
}

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebookFromFile(const api::RoadGeometry* road_geometry,
                                                                   const std::string& filename,
                                                                   const RuleRegistry& rule_registry) {
  return BuildFrom(road_geometry, YAML::LoadFile(filename), rule_registry);
}

}  // namespace maliput
