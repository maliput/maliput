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
/// When environment variable isn't set the list will be returned empty.
/// @param env_var Environemnt variable.
/// @returns A list of paths extracted from `env_var`.
std::vector<std::string> GetAllPathsFromEnvironment(const std::string& env_var);

}  // namespace utility
}  // namespace maliput
