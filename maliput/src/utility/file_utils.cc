// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
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
    maliput::log()->warn("Env var '{}' isn't set.", env_var);
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
