#pragma once

#include <memory>
#include <string>

#include "maliput/api/rules/rule_registry.h"

namespace maliput {

/// Instantiates an api::rules::RuleRegistry based on the specified @p input.
///
/// @param input The YAML RuleRegistry document.
///
/// @return An api::rules::RuleRegistry containing the rule types for DiscreteValueRule
/// and RangeValueRule Types specified in @p input.
///
/// @throws std::exception if the YAML document within @p input is invalid.
std::unique_ptr<api::rules::RuleRegistry> LoadRuleRegistry(const std::string& input);

/// Instantiates an api::rules::RuleRegistry based on the specified @p filename.
///
/// @param filename The path to the YAML RuleRegistry document.
///
/// @return An api::rules::RuleRegistry containing the rule types for DiscreteValueRule
/// and RangeValueRule Types specified in the file.
///
/// @throws std::exception if the YAML document in @p filename is invalid.
std::unique_ptr<api::rules::RuleRegistry> LoadRuleRegistryFromFile(const std::string& filename);

}  // namespace maliput
