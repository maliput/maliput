// Copyright 2021 Toyota Research Institute
// @file
// This file is a basic maliput plugin implementation which aims to
// be a test tool in order to verify the correct behaviour of MaliputManager class.
// This file shall not be modified.

#include <string>

#include "maliput/plugin/maliput_plugin_type.h"

namespace maliput {
namespace plugin {

// Implementation of these methods is a requirement of MaliputManager.
// @{
extern "C" std::string GetMaliputPluginId() { return std::string("dumb_plugin_1"); }
extern "C" maliput::plugin::MaliputPluginType GetMaliputPluginType() {
  return maliput::plugin::MaliputPluginType::kRoadNetworkLoader;
}
// }@

// Custom function callable from MaliputManager::ExecuteSymbol method.
extern "C" int MultiplyIntegers(int a, int b) { return a * b; };

}  // namespace plugin
}  // namespace maliput
