// Copyright 2021 Toyota Research Institute
#include "maliput/utility/filesystem.h"

#include <iostream>
// TODO(francocipollone): Remove "experimental" namespace once a newer version of GCC is used(8 or above).
#include <experimental/filesystem>

namespace maliput {
namespace utility {
namespace fs = std::experimental::filesystem;

namespace {

// Extracts the extension of a file.
// @param filepath Path to a file.
// @returns The extension of the file.
std::string GetExtensionFromPath(const std::string& filepath) {
  auto it = filepath.find_last_of('.');
  return it == std::string::npos ? filepath : filepath.substr(it + 1);
}

}  // namespace

std::vector<std::string> GetAllFilepathsFromDirectory(const std::string& directory_path,
                                                      const std::optional<std::string>& extension) {
  std::vector<std::string> filepaths{};
  for (const auto& entry : fs::directory_iterator(directory_path)) {
    if (fs::is_directory(entry)) {
      const auto more_paths = GetAllFilepathsFromDirectory(entry.path(), extension);
      filepaths.insert(filepaths.end(), more_paths.begin(), more_paths.end());
      continue;
    }
    if (extension.has_value() && !extension.value().empty()) {
      if (GetExtensionFromPath(entry.path()) != extension) {
        continue;
      }
    }
    filepaths.push_back(entry.path());
  }
  return std::move(filepaths);
}

std::vector<std::string> GetAllPathsFromEnvVar(const std::string& env_var) {
  std::istringstream path_stream(std::string(std::getenv(env_var.c_str())));
  const std::string delimeter{":"};
  std::vector<std::string> paths;
  std::string path;
  while (std::getline(path_stream, path, ':')) {
    paths.push_back(path);
  }
  return paths;
}

}  // namespace utility
}  // namespace maliput
