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
