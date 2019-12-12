#include "maliput/base/yaml_conversion.h"

namespace YAML {

Node convert<maliput::api::SRange>::encode(const maliput::api::SRange& rhs) {
  Node node;
  node.push_back(rhs.s0());
  node.push_back(rhs.s1());
  return node;
}

bool convert<maliput::api::SRange>::decode(const Node& node, maliput::api::SRange& rhs) {
  if (!node.IsSequence() || node.size() != 2) {
    return false;
  }
  rhs.set_s0(node[0].as<double>());
  rhs.set_s1(node[1].as<double>());
  return true;
}

}  // namespace YAML
