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
  std::unordered_map<MaliputPlugin::Id, MaliputPluginType> ListPlugins() const;

 private:
  // Environment variable name that holds the path to look for plugins.
  static constexpr char const* kMaliputPluginPathEnv{"MALIPUT_PLUGIN_PATH"};

  // Holds the loaded plugins.
  std::unordered_map<MaliputPlugin::Id, std::unique_ptr<MaliputPlugin>> plugins_;
};

}  // namespace plugin
}  // namespace maliput
