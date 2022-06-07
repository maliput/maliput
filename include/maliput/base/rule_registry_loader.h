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
/// @throws std::exception if the YAML document referred by @p filename is invalid.
std::unique_ptr<api::rules::RuleRegistry> LoadRuleRegistryFromFile(const std::string& filename);

}  // namespace maliput
