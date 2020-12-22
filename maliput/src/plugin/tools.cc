#include "tools.h"

namespace maliput {
namespace plugin {

std::string GetPathToLib(const std::string& lib_name) {
  const std::string PluginLibDir = MALIPUT_PLUGIN_LIBDIR;
  const std::string PluginLibPath = PluginLibDir + "/" + lib_name;
  return PluginLibPath;
}

}  // namespace plugin
}  // namespace maliput