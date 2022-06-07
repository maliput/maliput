// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput/base/traffic_light_book_loader.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/rules/traffic_light_book.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/base/traffic_light_book.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/math/quaternion.h"
#include "maliput/math/vector.h"

using maliput::api::InertialPosition;
using maliput::api::Rotation;
using maliput::api::rules::Bulb;
using maliput::api::rules::BulbColor;
using maliput::api::rules::BulbGroup;
using maliput::api::rules::BulbState;
using maliput::api::rules::BulbType;
using maliput::api::rules::TrafficLight;
using maliput::api::rules::UniqueBulbId;

namespace YAML {

template <>
struct convert<InertialPosition> {
  static Node encode(const InertialPosition& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, InertialPosition& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }
    rhs.set_x(node[0].as<double>());
    rhs.set_y(node[1].as<double>());
    rhs.set_z(node[2].as<double>());
    return true;
  }
};

template <>
struct convert<Rotation> {
  static Node encode(const Rotation& rhs) {
    const maliput::math::Quaternion& q = rhs.quat();
    Node node;
    node.push_back(q.w());
    node.push_back(q.x());
    node.push_back(q.y());
    node.push_back(q.z());
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, Rotation& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }
    const maliput::math::Quaternion q(node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),
                                      node[3].as<double>());
    rhs.set_quat(q);
    return true;
  }
};

template <>
struct convert<Bulb::BoundingBox> {
  static Node encode(const Bulb::BoundingBox& rhs) {
    Node min_node;
    min_node.push_back(rhs.p_BMin.x());
    min_node.push_back(rhs.p_BMin.y());
    min_node.push_back(rhs.p_BMin.z());

    Node max_node;
    max_node.push_back(rhs.p_BMax.x());
    max_node.push_back(rhs.p_BMax.y());
    max_node.push_back(rhs.p_BMax.z());

    Node node;
    node["min"] = min_node;
    node["max"] = max_node;

    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, Bulb::BoundingBox& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    const Node& min_node = node["min"];
    if (!min_node.IsDefined() || !min_node.IsSequence() || min_node.size() != 3) {
      return false;
    }
    const Node& max_node = node["max"];
    if (!max_node.IsDefined() || !max_node.IsSequence() || max_node.size() != 3) {
      return false;
    }
    rhs.p_BMin = maliput::math::Vector3(min_node[0].as<double>(), min_node[1].as<double>(), min_node[2].as<double>());
    rhs.p_BMax = maliput::math::Vector3(max_node[0].as<double>(), max_node[1].as<double>(), max_node[2].as<double>());
    return true;
  }
};

template <>
struct convert<BulbColor> {
  static Node encode(const BulbColor& rhs) {
    Node node;
    node.push_back(maliput::api::rules::BulbColorMapper().at(rhs));
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, BulbColor& rhs) {
    const std::string color = node.as<std::string>();
    bool result = false;
    for (const auto& it : maliput::api::rules::BulbColorMapper()) {
      if (it.second == color) {
        rhs = it.first;
        result = true;
      }
    }
    return result;
  }
};

template <>
struct convert<BulbType> {
  static Node encode(const BulbType& rhs) {
    Node node;
    node.push_back(maliput::api::rules::BulbTypeMapper().at(rhs));
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, BulbType& rhs) {
    const std::string type = node.as<std::string>();
    bool result = false;
    for (const auto& it : maliput::api::rules::BulbTypeMapper()) {
      if (it.second == type) {
        rhs = it.first;
        result = true;
      }
    }
    return result;
  }
};

template <>
struct convert<std::vector<BulbState>> {
  static Node encode(const std::vector<BulbState>& rhs) {
    Node node;
    const auto mapper = maliput::api::rules::BulbStateMapper();
    for (const auto& state : rhs) {
      node.push_back(mapper.at(state));
    }
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, std::vector<BulbState>& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    const auto mapper = maliput::api::rules::BulbStateMapper();
    for (const YAML::Node& state_node : node) {
      const std::string state = state_node.as<std::string>();
      bool result = false;
      for (const auto& it : mapper) {
        if (it.second == state) {
          rhs.push_back(it.first);
          result = true;
        }
      }
      if (!result) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace YAML

namespace maliput {
namespace {

std::unique_ptr<Bulb> BuildBulb(const YAML::Node& bulb_node) {
  MALIPUT_THROW_UNLESS(bulb_node.IsDefined());
  MALIPUT_THROW_UNLESS(bulb_node.IsMap());
  MALIPUT_THROW_UNLESS(bulb_node["ID"].IsDefined());
  const Bulb::Id id(bulb_node["ID"].as<std::string>());

  const YAML::Node& pose_node = bulb_node["Pose"];
  MALIPUT_THROW_UNLESS(pose_node.IsDefined());
  MALIPUT_THROW_UNLESS(pose_node.IsMap());
  MALIPUT_THROW_UNLESS(pose_node["position_bulb_group"].IsDefined());
  MALIPUT_THROW_UNLESS(pose_node["orientation_bulb_group"].IsDefined());
  const InertialPosition position_bulb_group = pose_node["position_bulb_group"].as<InertialPosition>();
  const Rotation orientation_bulb_group = pose_node["orientation_bulb_group"].as<Rotation>();

  const YAML::Node& bounding_box_node = bulb_node["BoundingBox"];
  Bulb::BoundingBox bounding_box;
  if (bounding_box_node.IsDefined()) {
    MALIPUT_THROW_UNLESS(bounding_box_node.IsMap());
    bounding_box = bounding_box_node.as<Bulb::BoundingBox>();
  }

  const YAML::Node& color_node = bulb_node["Color"];
  MALIPUT_THROW_UNLESS(color_node.IsDefined());
  const BulbColor color = color_node.as<BulbColor>();

  const YAML::Node& type_node = bulb_node["Type"];
  MALIPUT_THROW_UNLESS(type_node.IsDefined());
  const BulbType type = type_node.as<BulbType>();

  std::optional<double> arrow_orientation_rad = std::nullopt;
  if (type == BulbType::kArrow) {
    const YAML::Node& arrow_orientation_node = bulb_node["ArrowOrientation"];
    MALIPUT_THROW_UNLESS(arrow_orientation_node.IsDefined());
    arrow_orientation_rad = arrow_orientation_node.as<double>();
  }

  const YAML::Node& states_node = bulb_node["States"];
  std::vector<BulbState> bulb_states({BulbState::kOn, BulbState::kOff});
  if (states_node.IsDefined()) {
    bulb_states = states_node.as<std::vector<BulbState>>();
  }

  return std::make_unique<Bulb>(id, position_bulb_group, orientation_bulb_group, color, type, arrow_orientation_rad,
                                bulb_states, bounding_box);
}

std::unique_ptr<BulbGroup> BuildBulbGroup(const YAML::Node& bulb_group_node) {
  MALIPUT_THROW_UNLESS(bulb_group_node.IsDefined());
  MALIPUT_THROW_UNLESS(bulb_group_node.IsMap());
  MALIPUT_THROW_UNLESS(bulb_group_node["ID"].IsDefined());
  const BulbGroup::Id id(bulb_group_node["ID"].as<std::string>());
  const YAML::Node& pose_node = bulb_group_node["Pose"];
  MALIPUT_THROW_UNLESS(pose_node.IsDefined());
  MALIPUT_THROW_UNLESS(pose_node.IsMap());
  MALIPUT_THROW_UNLESS(pose_node["position_traffic_light"].IsDefined());
  MALIPUT_THROW_UNLESS(pose_node["orientation_traffic_light"].IsDefined());
  const InertialPosition position_traffic_light = pose_node["position_traffic_light"].as<InertialPosition>();
  const Rotation orientation_traffic_light = pose_node["orientation_traffic_light"].as<Rotation>();
  const YAML::Node& bulbs_node = bulb_group_node["Bulbs"];
  MALIPUT_THROW_UNLESS(bulbs_node.IsDefined());
  MALIPUT_THROW_UNLESS(bulbs_node.IsSequence());
  std::vector<std::unique_ptr<Bulb>> bulbs;
  for (const YAML::Node& bulb_node : bulbs_node) {
    bulbs.push_back(BuildBulb(bulb_node));
  }
  return std::make_unique<BulbGroup>(id, position_traffic_light, orientation_traffic_light, std::move(bulbs));
}

std::unique_ptr<TrafficLight> BuildTrafficLight(const YAML::Node& traffic_light_node) {
  MALIPUT_THROW_UNLESS(traffic_light_node.IsMap());
  const TrafficLight::Id id(traffic_light_node["ID"].as<std::string>());
  const YAML::Node& pose_node = traffic_light_node["Pose"];
  MALIPUT_THROW_UNLESS(pose_node.IsDefined());
  MALIPUT_THROW_UNLESS(pose_node.IsMap());
  MALIPUT_THROW_UNLESS(pose_node["position_road_network"].IsDefined());
  MALIPUT_THROW_UNLESS(pose_node["orientation_road_network"].IsDefined());
  const InertialPosition position_road_network = pose_node["position_road_network"].as<InertialPosition>();
  const Rotation orientation_road_network = pose_node["orientation_road_network"].as<Rotation>();

  const YAML::Node& bulb_groups_node = traffic_light_node["BulbGroups"];
  MALIPUT_THROW_UNLESS(bulb_groups_node.IsDefined());
  MALIPUT_THROW_UNLESS(bulb_groups_node.IsSequence());
  std::vector<std::unique_ptr<BulbGroup>> bulb_groups;
  for (const YAML::Node& bulb_group_node : bulb_groups_node) {
    bulb_groups.push_back(BuildBulbGroup(bulb_group_node));
  }
  return std::make_unique<TrafficLight>(id, position_road_network, orientation_road_network, std::move(bulb_groups));
}

std::unique_ptr<api::rules::TrafficLightBook> BuildFrom(const YAML::Node& root_node) {
  MALIPUT_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& traffic_lights_node = root_node["TrafficLights"];
  MALIPUT_THROW_UNLESS(traffic_lights_node.IsDefined());
  MALIPUT_THROW_UNLESS(traffic_lights_node.IsSequence());
  auto result = std::make_unique<TrafficLightBook>();
  for (const YAML::Node& traffic_light_node : traffic_lights_node) {
    result->AddTrafficLight(BuildTrafficLight(traffic_light_node));
  }
  return result;
}

}  // namespace

std::unique_ptr<api::rules::TrafficLightBook> LoadTrafficLightBook(const std::string& input) {
  return BuildFrom(YAML::Load(input));
}

std::unique_ptr<api::rules::TrafficLightBook> LoadTrafficLightBookFromFile(const std::string& filename) {
  return BuildFrom(YAML::LoadFile(filename));
}

}  // namespace maliput
