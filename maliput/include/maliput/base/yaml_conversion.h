#pragma once

#include "yaml-cpp/yaml.h"

#include "maliput/api/regions.h"

namespace YAML {

/// Struct used for encoding and decoding api::SRange with a YAML::Node.
template <>
struct convert<maliput::api::SRange> {
  static Node encode(const maliput::api::SRange& rhs);
  static bool decode(const Node& node, maliput::api::SRange& rhs);
};

}  // namespace YAML

namespace maliput {

/// Constants to identify attributes of DiscreteValueRule Types and DiscreteValueRule Types.
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

/// Label to identify rule type.
enum class RuleType {
  /// Label for a DiscreteValueRule Type.
  kDiscreteValueRuleType,
  /// Label for a RangeValueRule Type.
  kRangeValueRuleType,
  /// Label for an unidentified rule type.
  kUnknownRuleType,
};

/// Determines whether the `rule_node` corresponds to a api::rules::DiscreteValueRule::DiscreteValue description.
/// @param rule_node A YAML::Node that contains information of a api::rules::DiscreteValueRule::DiscreteValue.
/// @return True if a api::rules::DiscreteValueRule::DiscreteValue is contained in `rule_node`.
bool IsDiscreteValue(const YAML::Node& rule_node);

/// Determines whether the `rule_node` corresponds to a api::rules::RangeValueRule::Range description.
/// @param rule_node A YAML::Node that contains information of a api::rules::RangeValueRule::Range.
/// @return True if a api::rules::RangeValueRule::Range is contained in `rule_node`.
bool IsRangeValue(const YAML::Node& rule_node);

/// Returns the severity field value from the `node`.
/// If severity is undefined, It will return the strictest value.
/// @param node A YAML::Node that contains severity of the api::rules::Rule::State.
/// @return A value indicating severityness. 0 is the strictest.
///
/// @throws maliput::common::assertion_error if the severity is negative.
int GetSeverityFromYamlNode(const YAML::Node& node);

/// Get min and max values from the yaml node.
/// @param node A YAML::Node that contains the range of the api::rules::RangeValueRule::Range.
/// @return The min and max values contained in the range from the node.
///
/// @throws maliput::common::assertion_error if the range is ill-defined.
std::pair<double, double> GetRangeMinMaxValuesFromYamlNode(const YAML::Node& node);

/// Get the value of a api::rules::DiscreteValueRule::DiscreteValue from a yaml node.
/// @param node A YAML::Node that contains the value of the api::rules::DiscreteValueRule::DiscreteValue.
/// @return A std::string contained in the value from the `node`.
///
/// @throws maliput::common::assertion_error if the value is ill-defined.
std::string GetValueFromYamlNode(const YAML::Node& node);

/// Get the description of a api::rules::RangeValueRule::Range from a yaml node.
/// @param node A YAML::Node that contains the description of the api::rules::DiscreteValueRule::Range.
/// @return A std::string contained in the description field value from the `node`.
///
/// @throws maliput::common::assertion_error if the description is ill-defined.
std::string GetDescriptionFromYamlNode(const YAML::Node& node);

}  // namespace maliput
