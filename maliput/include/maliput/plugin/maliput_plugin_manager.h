// Copyright 2021 Toyota Research Institute
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "maliput/common/maliput_copyable.h"
#include "maliput/plugin/maliput_plugin.h"

namespace maliput {
namespace plugin {

/// Manages the lifecycle of `MaliputPlugins`.
/// Upon creation, it will try to load all the available plugins in path and made them available
/// via GetPlugin().
class MaliputPluginManager {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MaliputPluginManager);

  /// Constructs a MaliputPluginManager.
  /// It looks for plugins in the paths that are described in the environment variable called `MALIPUT_PLUGIN_PATH`.
  /// Plugins will not be loaded twice and their uniqueness is determined by their id.
  /// When a duplicate is found, the most recently found plugin prevails.
  MaliputPluginManager();

  /// Get a pointer to an already loaded plugin.
  /// @param id Is the MaliputPlugin::Id of the plugin to get.
  /// @returns A MaliputPlugin pointer to the requested plugin. Returns nullptr when the plugin wasn't found.
  const MaliputPlugin* GetPlugin(const MaliputPlugin::Id& id) const;

  /// Loads a new plugin. If a plugin with the same id was already loaded then it is replaced by the new one.
  /// @param path_to_plugin Path to new the plugin.
  void AddPlugin(const std::string& path_to_plugin);

  /// @returns A vector with the MaliputPlugin::Id of the loaded plugins.
  std::vector<MaliputPlugin::Id> ListPlugins();

 private:
  // Environment variable name that holds the path to look for plugins.
  static constexpr char const* kMaliputPluginPathEnv{"MALIPUT_PLUGIN_PATH"};

  // Holds the loaded plugins.
  std::unordered_map<MaliputPlugin::Id, std::unique_ptr<MaliputPlugin>> plugins_;
};

}  // namespace plugin
}  // namespace maliput
