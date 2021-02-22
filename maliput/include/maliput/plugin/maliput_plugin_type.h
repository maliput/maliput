// Copyright 2021 Toyota Research Institute
#pragma once

namespace maliput {
namespace plugin {

/// Types of maliput plugin.
/// Note: All function/enum that have extern "C" linkage share the same space of names.
///       Although this enum is within `maliput::plugin` namespace you can't have any
///       other extern "C" enum with name `MaliputPluginType` in any other namespace.
extern "C" typedef enum MaliputPluginType {
  kRoadNetworkLoader,
} MaliputPluginType;

}  // namespace plugin
}  // namespace maliput
