// Copyright 2021 Toyota Research Institute
#include "maliput/utility/file_utils.h"

#include <iostream>
// TODO(#379): Remove "experimental" namespace once a newer version of GCC is used(8 or above).
#include <experimental/filesystem>

#include "maliput/common/logger.h"

namespace maliput {
namespace utility {
namespace fs = std::experimental::filesystem;

std::string GetSuffixFromPath(const std::string& filepath) {
  auto it = filepath.find_last_of('.');
  return it == std::string::npos ? filepath : filepath.substr(it + 1);
}

std::vector<std::string> GetAllFilePathsFromDirectory(const std::string& directory_path,
                                                      const std::optional<std::string>& ends_with_suffix) {
  std::vector<std::string> filepaths{};
  for (const auto& entry : fs::directory_iterator(directory_path)) {
    if (fs::is_directory(entry)) {
      const auto more_paths = GetAllFilePathsFromDirectory(entry.path(), ends_with_suffix);
      filepaths.insert(filepaths.end(), more_paths.begin(), more_paths.end());
      continue;
    }
    if (ends_with_suffix.has_value() && !ends_with_suffix.value().empty()) {
      if (GetSuffixFromPath(entry.path()) != ends_with_suffix) {
        continue;
      }
    }
    filepaths.push_back(entry.path());
  }
  return filepaths;
}

std::vector<std::string> GetAllPathsFromEnvironment(const std::string& env_var) {
  char* env = std::getenv(env_var.c_str());
  if (env == nullptr) {
    maliput::log()->warn("Env var '{}' isn't set", env_var);
    return {};
  }
  std::istringstream path_stream(env);
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
