#pragma once

#include "yaml-cpp/yaml.h"

#include "maliput/api/regions.h"

namespace YAML {

/// Struct used for encoding and decoding api::SRange with a YAML::Node.
template <>
struct convert<maliput::api::SRange> {
  static Node encode(const maliput::api::SRange& rhs);
  static bool decode(const Node& node, maliput::api::SRange& rhs);
};

}  // namespace YAML
