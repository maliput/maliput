// Copyright 2021 Toyota Research Institute
#include "maliput/plugin/maliput_plugin.h"

namespace maliput {
namespace plugin {

MaliputPlugin::MaliputPlugin(const std::string& path_to_lib) {
  MALIPUT_THROW_UNLESS(!path_to_lib.empty());
  // Call dlerror() before dlopen(~) to ensure that we get the most recent error value.
  dlerror();
  lib_handle_.reset(dlopen(path_to_lib.c_str(), RTLD_LAZY | RTLD_LOCAL));
  if (!lib_handle_) {
    MALIPUT_THROW_MESSAGE("Cannot load library: " + std::string(dlerror()));
  }
  id_ = MaliputPlugin::Id(ExecuteSymbol<std::string>(kMaliputPluginIdSym));
  type_ = ExecuteSymbol<MaliputPluginType>(kMaliputPluginTypeSym);
}

}  // namespace plugin
}  // namespace maliput
