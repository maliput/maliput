#pragma once

#include <memory>
#include <string>

#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule_registry.h"

namespace maliput {

/// Instantiates an api::rules::RoadRulebook based on the specified @p
/// road_geometry and @p input.
///
/// @param road_geometry The road geometry to which the api::rules::RoadRulebook
/// to be loaded applies.
///
/// @param input The YAML RoadRulebook document.
///
/// @return An api::rules::RoadRulebook containing the rules specified in
/// @p input.
///
/// @throws std::exception if the YAML document within @p input is invalid, or
/// an api::rules::RightOfWayRule within @p input has an invalid state or zone
/// type.
std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebook(const api::RoadGeometry* road_geometry,
                                                           const std::string& input);

/// Instantiates an api::rules::RoadRulebook based on the specified @p
/// road_geometry and @p filename.
///
/// @param road_geometry The road geometry to which the api::rules::RoadRulebook
/// to be loaded applies.
///
/// @param filename The path to the YAML RoadRulebook document.
///
/// @return An api::rules::RoadRulebook containing the rules specified in
/// @p input.
///
/// @throws std::exception if the YAML document in @p filename is invalid, or
/// an api::rules::RightOfWayRule within that document has an invalid state or zone
/// type.
std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebookFromFile(const api::RoadGeometry* road_geometry,
                                                                   const std::string& filename);

/// Instantiates an api::rules::RoadRulebook based on the specified @p
/// road_geometry, @p input and @p rule_registry.
///
/// @param road_geometry The road geometry to which the api::rules::RoadRulebook
/// to be loaded applies. It must not be nullptr.
///
/// @param input The YAML RoadRulebook document.
///
/// @param rule_registry An api::rules::RuleRegistry for creating allowed rule types.
///
/// @return An api::rules::RoadRulebook containing the rules specified in
/// @p input.
///
/// @throws std::exception if the YAML document in @p input is invalid.
std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebook(const api::RoadGeometry* road_geometry,
                                                           const std::string& input,
                                                           const api::rules::RuleRegistry& rule_registry);

/// Instantiates an api::rules::RoadRulebook based on the specified @p
/// road_geometry, @p filename and @p rule_registry.
///
/// @param road_geometry The road geometry to which the api::rules::RoadRulebook
/// to be loaded applies. It must not be nullptr.
///
/// @param filename The path to the YAML RoadRulebook document.
///
/// @param rule_registry An api::rules::RuleRegistry for creating allowed rule types.
///
/// @return An api::rules::RoadRulebook containing the rules specified in
/// @p input.
///
/// @throws std::exception if the YAML document in @p filename is invalid.
std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebookFromFile(const api::RoadGeometry* road_geometry,
                                                                   const std::string& filename,
                                                                   const api::rules::RuleRegistry& rule_registry);

}  // namespace maliput
