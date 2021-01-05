// Copyright 2021 Toyota Research Institute
#include "maliput/plugin/maliput_plugin.h"

namespace maliput {
namespace plugin {

MaliputPlugin::MaliputPlugin(void* lib_handle) : lib_handle_(lib_handle) {
  MALIPUT_THROW_UNLESS(lib_handle_ != nullptr);

  id_ = MaliputPluginId(ExecuteSymbol<std::string>(kMaliputPluginIdSym));
  type_ = ExecuteSymbol<MaliputPlugin::Type>(kMaliputPluginTypeSym);
}

}  // namespace plugin
}  // namespace maliput
