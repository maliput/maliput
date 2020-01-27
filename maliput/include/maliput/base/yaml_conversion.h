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

/// Constants to identify attributes of api::rules::DiscreteValueRule Types.
struct DiscreteValueRuleConstants {
  static const char* kValue;
  static const char* kValues;
};

/// Constants to identify attributes of api::rules::RangeValueRule Types.
struct RangeValueRuleConstants {
  static const char* kDescription;
  static const char* kRange;
  static const char* kRanges;
};

/// Constants to identify attributes used in both api::rules::DiscreteValueRules and api::rules::RangeValueRules Types.
struct RuleConstants {
  static const char* kId;
  static const char* kLaneId;
  static const char* kRelatedRules;
  static const char* kRelatedUniqueIds;
  static const char* kSeverity;
  static const char* kSRange;
  static const char* kType;
  static const char* kZone;
};

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
/// @return True when a api::rules::DiscreteValueRule::DiscreteValue is contained in `rule_node`.
bool IsDiscreteValue(const YAML::Node& rule_node);

/// Determines whether the `rule_node` corresponds to a api::rules::RangeValueRule::Range description.
/// @param rule_node A YAML::Node that contains information of a api::rules::RangeValueRule::Range.
/// @return True when a api::rules::RangeValueRule::Range is contained in `rule_node`.
bool IsRangeValue(const YAML::Node& rule_node);

/// Returns the severity field value from the `node`.
/// When severity is undefined, it will return api::rules::Rule::State::kStrict value.
/// @param node A YAML::Node that contains severity of the api::rules::Rule::State.
/// @return A value indicating severityness. 0 is the strictest.
///
/// @throws maliput::common::assertion_error when the severity is negative.
int GetSeverityFromYamlNode(const YAML::Node& node);

/// Get min and max values from the yaml node.
/// @param node A YAML::Node that contains the range of the api::rules::RangeValueRule::Range.
/// @return The min and max values contained in the range from the node.
///
/// @throws maliput::common::assertion_error when the range is ill-defined.
/// @throws maliput::common::assertion_error when the min value is greater than max value.
std::pair<double, double> GetRangeMinMaxValuesFromYamlNode(const YAML::Node& node);

/// Get the value of a api::rules::DiscreteValueRule::DiscreteValue from a yaml node.
/// @param node A YAML::Node that contains the value of the api::rules::DiscreteValueRule::DiscreteValue.
/// @return A std::string contained in the value from the `node`.
///
/// @throws maliput::common::assertion_error when the value is ill-defined.
std::string GetValueFromYamlNode(const YAML::Node& node);

/// Get the description of a api::rules::RangeValueRule::Range from a yaml node.
/// @param node A YAML::Node that contains the description of the api::rules::DiscreteValueRule::Range.
/// @return A std::string contained in the description field value from the `node`.
///
/// @throws maliput::common::assertion_error when the description is ill-defined.
std::string GetDescriptionFromYamlNode(const YAML::Node& node);

}  // namespace maliput
