// Copyright 2021 Toyota Research Institute
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace maliput {
namespace utility {

/// Get all filepaths from a particular directory.
/// @param directory_path Path to the directory.
/// @param extension When it is not nullopt, filepaths must end with the `extension` value.
/// @returns The paths to all the files within `directory_path`.
///          The returned paths are absolute if the `directory_path` is an absolute path,
///          otherwise it will return relative paths.
std::vector<std::string> GetAllFilepathsFromDirectory(const std::string& directory_path,
                                                      const std::optional<std::string>& extension);

/// Retrieves a list of paths that live in a environment variable.
/// @param env_var Environemnt variable.
/// @returns A list of paths extracted from `env_var`.
std::vector<std::string> GetAllPathsFromEnvVar(const std::string& env_var);

}  // namespace utility
}  // namespace maliput
