// Copyright 2021 Toyota Research Institute
#include "maliput/plugin/maliput_plugin.h"

namespace maliput {
namespace plugin {

MaliputPlugin::MaliputPlugin(const std::string& path_to_lib) {
  MALIPUT_THROW_UNLESS(!path_to_lib.empty());
  // Call dlerror() before dlopen(~) to ensure that we get accurate error
  // reporting afterwards.
  dlerror();
  lib_handle_ = std::unique_ptr<void, std::function<void(void*)>>(dlopen(path_to_lib.c_str(), RTLD_LAZY),
                                                                  [](void* ptr) { dlclose(ptr); });
  if (!lib_handle_) {
    MALIPUT_THROW_MESSAGE("Cannot load library: " + static_cast<std::string>(dlerror()));
  }
  id_ = MaliputPluginId(ExecuteSymbol<std::string>(kMaliputPluginIdSym));
  type_ = ExecuteSymbol<MaliputPluginType>(kMaliputPluginTypeSym);
}

}  // namespace plugin
}  // namespace maliput
