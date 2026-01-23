// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include <dlfcn.h>

#include <cstring>

#include <link.h>

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
  maliput::log()->debug("'", env_var, "' env var contains ", paths_from_env.size(), " paths:");
  for (const auto& path_from_env : paths_from_env) {
    maliput::log()->debug("\t'", path_from_env, "'");
    const maliput::common::Path path{path_from_env};
    if (!path.is_directory()) {
      maliput::log()->debug("The path '", path_from_env, "' isn't a valid directory for the ", env_var,
                            " env var, omitting...");
      continue;
    }
    const auto filepaths_from_dir = maliput::utility::GetAllFilePathsFromDirectory(path.get_path(), "so");
    filepaths.insert(filepaths.end(), filepaths_from_dir.begin(), filepaths_from_dir.end());
  }
  return filepaths;
}

// Structure to hold search parameters and results for dl_iterate_phdr callback.
struct PluginSearchContext {
  std::string search_pattern;  // Pattern to match in library path (e.g., "maliput_malidrive_road_network")
  std::string found_path;      // Full path of the found library
  bool found{false};
};

// Callback for dl_iterate_phdr() to find loaded libraries matching a pattern.
// @param info Information about the shared object.
// @param size Size of the info structure.
// @param data User-provided context (PluginSearchContext*).
// @return 0 to continue iteration, non-zero to stop.
int FindLoadedPluginCallback(struct dl_phdr_info* info, size_t /* size */, void* data) {
  auto* context = static_cast<PluginSearchContext*>(data);
  if (info->dlpi_name != nullptr && std::strlen(info->dlpi_name) > 0) {
    const std::string lib_path(info->dlpi_name);
    // Check if the library path contains the search pattern.
    if (lib_path.find(context->search_pattern) != std::string::npos) {
      context->found_path = lib_path;
      context->found = true;
      return 1;  // Stop iteration
    }
  }
  return 0;  // Continue iteration
}

// Finds a pre-loaded library by matching a pattern in the library name.
// Uses dl_iterate_phdr() to scan all loaded shared objects.
// @param pattern The pattern to search for in library names (e.g., "maliput_malidrive_road_network").
// @return The full path to the loaded library if found, empty string otherwise.
std::string FindLoadedLibraryByPattern(const std::string& pattern) {
  PluginSearchContext context;
  context.search_pattern = pattern;
  dl_iterate_phdr(FindLoadedPluginCallback, &context);
  return context.found ? context.found_path : "";
}

}  // namespace

// List of known plugin base names (without lib prefix, version suffixes, or .so extension)
// that may be pre-loaded as ELF NEEDED dependencies.
// These patterns will be matched against loaded library paths using dl_iterate_phdr().
const std::vector<std::string> kPreloadedPluginPatterns{
    // See https://github.com/maliput/maliput_malidrive/
    "maliput_malidrive_road_network",
};

MaliputPluginManager::MaliputPluginManager() {
  // Check if any known plugins are already loaded in memory (e.g., as ELF NEEDED dependencies).
  // Using dl_iterate_phdr() to find libraries by pattern, handling version suffixes like "-c2dfc143".
  for (const auto& plugin_pattern : kPreloadedPluginPatterns) {
    const std::string loaded_lib_path = FindLoadedLibraryByPattern(plugin_pattern);
    if (!loaded_lib_path.empty()) {
      maliput::log()->info("Found pre-loaded plugin matching '", plugin_pattern, "': ", loaded_lib_path);
      // Use RTLD_NOLOAD to get a handle to the already-loaded library without incrementing refcount issues.
      void* existing_handle = dlopen(loaded_lib_path.c_str(), RTLD_LAZY | RTLD_LOCAL | RTLD_NOLOAD);
      if (existing_handle != nullptr) {
        // Create MaliputPlugin directly from the existing handle (ownership transferred).
        std::unique_ptr<MaliputPlugin> maliput_plugin = std::make_unique<MaliputPlugin>(existing_handle);
        const auto id = maliput_plugin->GetId();
        plugins_[MaliputPlugin::Id(id)] = std::move(maliput_plugin);
        maliput::log()->info("Plugin Id: ", id, " was loaded from existing handle.");
      } else {
        maliput::log()->warn("Found library '", loaded_lib_path, "' but failed to get handle: ", dlerror());
      }
    }
  }

  const auto library_paths = GetPluginLibraryPaths(kMaliputPluginPathEnv);
  for (const auto& path : library_paths) {
    AddPlugin(path);
  }
  maliput::log()->debug("Number of plugins loaded: ", plugins_.size());
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
  maliput::log()->debug("Adding plugin with Id: ", id, " from path: ", path_to_plugin);
  if (is_repeated) {
    plugins_.erase(MaliputPlugin::Id(id));
  }
  plugins_[MaliputPlugin::Id(id)] = std::move(maliput_plugin);
  maliput::log()->debug("Plugin loaded from path: ", path_to_plugin);
  maliput::log()->info((is_repeated ? "A new version of Plugin Id: " + id + " was loaded."
                                    : "Plugin Id: " + id + " was correctly loaded."));
}

std::unordered_map<MaliputPlugin::Id, MaliputPluginType> MaliputPluginManager::ListPlugins() const {
  std::unordered_map<MaliputPlugin::Id, MaliputPluginType> id_type{};
  for (const auto& plugin : plugins_) {
    id_type.emplace(plugin.first, plugin.second->GetType());
  }
  return id_type;
}

}  // namespace plugin
}  // namespace maliput
