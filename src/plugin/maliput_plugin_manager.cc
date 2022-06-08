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
#include "maliput/plugin/maliput_plugin_manager.h"

#include "maliput/common/filesystem.h"
#include "maliput/common/logger.h"
#include "maliput/utility/file_utils.h"

namespace maliput {
namespace plugin {
namespace {

// Looks for shared object(.so files) that are located in the paths that `env_var` is pointing to.
// It logs a warning message if a path isn't a valid directory path.
// @param env_var Environment variable.
// @returns A list of shared objects filepaths.
std::vector<std::string> GetPluginLibraryPaths(const std::string& env_var) {
  const auto paths_from_env = maliput::utility::GetAllPathsFromEnvironment(env_var);
  std::vector<std::string> filepaths{};
  maliput::log()->trace("'{}' env var contains {} paths:", env_var, paths_from_env.size());
  for (const auto& path_from_env : paths_from_env) {
    maliput::log()->trace("\t'{}'", path_from_env);
    const maliput::common::Path path{path_from_env};
    if (!path.is_directory()) {
      maliput::log()->warn("The path '{}' isn't a valid directory for the {} env var, omitting...", path_from_env,
                           env_var);
      continue;
    }
    const auto filepaths_from_dir = maliput::utility::GetAllFilePathsFromDirectory(path.get_path(), "so");
    filepaths.insert(filepaths.end(), filepaths_from_dir.begin(), filepaths_from_dir.end());
  }
  return filepaths;
}

}  // namespace

MaliputPluginManager::MaliputPluginManager() {
  const auto library_paths = GetPluginLibraryPaths(kMaliputPluginPathEnv);
  for (const auto& path : library_paths) {
    AddPlugin(path);
  }
  maliput::log()->info("Number of plugins loaded: {}", plugins_.size());
}

const MaliputPlugin* MaliputPluginManager::GetPlugin(const MaliputPlugin::Id& id) const {
  const auto it = plugins_.find(id);
  return it == plugins_.end() ? nullptr : it->second.get();
}

void MaliputPluginManager::AddPlugin(const std::string& path_to_plugin) {
  MALIPUT_THROW_UNLESS(!path_to_plugin.empty());
  std::unique_ptr<MaliputPlugin> maliput_plugin = std::make_unique<MaliputPlugin>(path_to_plugin);
  const auto id = maliput_plugin->GetId();
  const bool is_repeated{plugins_.find(MaliputPlugin::Id(id)) != plugins_.end()};
  plugins_[MaliputPlugin::Id(id)] = std::move(maliput_plugin);
  maliput::log()->info(
      (is_repeated ? "A new version of Plugin Id: {} was loaded." : "Plugin Id: {} was correctly loaded."), id);
}

std::vector<MaliputPlugin::Id> MaliputPluginManager::ListPlugins() {
  std::vector<MaliputPlugin::Id> keys{};
  for (const auto& plugin : plugins_) {
    keys.push_back(plugin.first);
  }
  return keys;
}

}  // namespace plugin
}  // namespace maliput
