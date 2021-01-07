// Copyright 2021 Toyota Research Institute

#include "maliput/plugin/maliput_plugin_manager.h"
#include "maliput/common/logger.h"
#include "maliput/utility/filesystem.h"

namespace maliput {
namespace plugin {
namespace {

// Looks for shared object(.so files) that are located in the paths that `env_var` is pointing to.
// @param env_var Environment variable.
// @returns A list of shared objects filepaths.
std::vector<std::string> GetPluginLibraryPaths(const std::string& env_var) {
  const auto paths_from_env = maliput::utility::GetAllPathsFromEnvVar(env_var);
  std::vector<std::string> filepaths{};
  for (const auto& path : paths_from_env) {
    const auto filepaths_from_dir = maliput::utility::GetAllFilepathsFromDirectory(path, "so");
    filepaths.insert(filepaths.end(), filepaths_from_dir.begin(), filepaths_from_dir.end());
  }
  return std::move(filepaths);
}

}  // namespace

MaliputPluginManager::MaliputPluginManager() {
  const auto library_paths = GetPluginLibraryPaths(kMaliputPluginPathEnv);
  for (const auto& path : library_paths) {
    AddPlugin(path);
  }
  maliput::log()->info("Number of plugins loaded: {}", plugins_.size());
}

const MaliputPlugin* MaliputPluginManager::GetPlugin(const MaliputPluginId& id) const {
  const auto it = plugins_.find(id);
  if (it == plugins_.end()) {
    return nullptr;
  }
  return it->second.get();
}

void MaliputPluginManager::AddPlugin(const std::string& path_to_plugin) {
  MALIPUT_THROW_UNLESS(!path_to_plugin.empty());
  std::unique_ptr<MaliputPlugin> maliput_plugin = std::make_unique<MaliputPlugin>(path_to_plugin);
  const auto id = maliput_plugin->GetId();
  if (plugins_.find(MaliputPluginId(id)) != plugins_.end()) {
    maliput::log()->debug("Plugin with Id: {} is already loaded.", id);
    return;
  }
  plugins_.emplace(MaliputPluginId(id), std::move(maliput_plugin));
}

}  // namespace plugin
}  // namespace maliput
