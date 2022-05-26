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
// Copyright 2021 Toyota Research Institute
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace maliput {
namespace utility {

/// Extracts the suffix of a file.
/// @param filepath Path to a file.
/// @returns The suffix of the file.
std::string GetSuffixFromPath(const std::string& filepath);

/// Get all filepaths from a particular directory.
/// @param directory_path Path to the directory.
/// @param ends_with_suffix When it is not nullopt, filepaths must end with the `ends_with_suffix` value.
/// @returns The paths to all the files within `directory_path`.
///          The returned paths are absolute if the `directory_path` is an absolute path,
///          otherwise it will return relative paths.
std::vector<std::string> GetAllFilePathsFromDirectory(const std::string& directory_path,
                                                      const std::optional<std::string>& ends_with_suffix);

/// Retrieves a list of paths that live in a environment variable.
/// When environment variable isn't set the list will be empty.
/// @param env_var Environemnt variable.
/// @returns A list of paths extracted from `env_var`.
std::vector<std::string> GetAllPathsFromEnvironment(const std::string& env_var);

}  // namespace utility
}  // namespace maliput
