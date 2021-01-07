// Copyright 2021 Toyota Research Institute
#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "maliput/common/maliput_copyable.h"
#include "maliput/plugin/maliput_plugin.h"

namespace maliput {
namespace plugin {

/// MaliputPluginManager is in charge of looking for all plugin libraries that are available to be loaded and
/// create a collection of MaliputPlugin that will manage the lifetime of the loaded library.
class MaliputPluginManager {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MaliputPluginManager);

  /// Constructs a MaliputPluginManager.
  /// It looks for plugins in the paths that are described in the environment variable called @ref
  /// kMaliputPluginPathEnv. All the plugins that are found will be loaded unless that the id of the plugin is repeated,
  /// in this case the plugin which was first loaded will remain active.
  MaliputPluginManager();

  /// Get a pointer to an already loaded plugin.
  /// @param id Is the MaliputPluginId of the plugin to get.
  /// @returns A MaliputPlugin pointer to the requested plugin. Returns nullptr when the plugin wasn't found.
  const MaliputPlugin* GetPlugin(const MaliputPluginId& id) const;

  /// Loads a new plugin. If a plugin with the same id was already loaded then it aborts the loading of the new plugin.
  /// @param path_to_plugin Path to new the plugin.
  void AddPlugin(const std::string& path_to_plugin);

 private:
  // Environment variable name that holds the path to look for plugins.
  static constexpr char const* kMaliputPluginPathEnv{"MALIPUT_PLUGIN_PATH"};

  // Holds the loaded plugins.
  std::unordered_map<MaliputPluginId, std::unique_ptr<MaliputPlugin>> plugins_;
};

}  // namespace plugin
}  // namespace maliput
