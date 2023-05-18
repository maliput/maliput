// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_object/loader/loader.h"

#include <map>
#include <string>
#include <utility>

#include <maliput/common/maliput_throw.h>
#include <maliput/math/bounding_box.h>
#include <yaml-cpp/yaml.h>

#include "maliput_object/api/object.h"
#include "maliput_object/base/manual_object_book.h"

namespace YAML {

template <>
struct convert<maliput::math::Vector3> {
  static Node encode(const maliput::math::Vector3& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    return node;
  }

  static bool decode(const Node& node, maliput::math::Vector3& rhs) {
    MALIPUT_THROW_UNLESS(node.IsSequence());
    MALIPUT_THROW_UNLESS(node.size() == 3);
    rhs.x() = node[0].as<double>();
    rhs.y() = node[1].as<double>();
    rhs.z() = node[2].as<double>();
    return true;
  }
};

template <>
struct convert<maliput::math::RollPitchYaw> {
  static Node encode(const maliput::math::RollPitchYaw& rhs) {
    Node node;
    node.push_back(rhs.roll_angle());
    node.push_back(rhs.pitch_angle());
    node.push_back(rhs.yaw_angle());
    return node;
  }

  static bool decode(const Node& node, maliput::math::RollPitchYaw& rhs) {
    MALIPUT_THROW_UNLESS(node.IsSequence());
    MALIPUT_THROW_UNLESS(node.size() == 3);
    rhs.set(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
    return true;
  }
};

}  // namespace YAML

namespace maliput {
namespace object {
namespace loader {
namespace {
// TODO(#15): Decide to pass it as a construction argument or read it from the input file.
constexpr const double kTolerance{1e-3};

std::unique_ptr<maliput::math::BoundingBox> ParseBoundingBox(const YAML::Node& node, double tolerance) {
  MALIPUT_THROW_UNLESS(node.IsMap());
  MALIPUT_THROW_UNLESS(node["position"].IsDefined());
  MALIPUT_THROW_UNLESS(node["rotation"].IsDefined());
  MALIPUT_THROW_UNLESS(node["type"].IsDefined() && node["type"].as<std::string>() == "box");
  MALIPUT_THROW_UNLESS(node["box_size"].IsDefined());

  const maliput::math::Vector3 position = node["position"].as<maliput::math::Vector3>();
  const maliput::math::RollPitchYaw& rotation = node["rotation"].as<maliput::math::RollPitchYaw>();
  const maliput::math::Vector3 box_size = node["box_size"].as<maliput::math::Vector3>();

  return std::make_unique<maliput::math::BoundingBox>(position, box_size, rotation, tolerance);
}

std::map<std::string, std::string> ParseProperties(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node.IsMap());
  std::map<std::string, std::string> properties;

  for (YAML::const_iterator property_it = node.begin(); property_it != node.end(); ++property_it) {
    properties.emplace(property_it->first.as<std::string>(), property_it->second.as<std::string>());
  }

  return properties;
}

std::unique_ptr<api::Object<maliput::math::Vector3>> ParseObject(const std::string& id, const YAML::Node& node,
                                                                 double tolerance) {
  MALIPUT_THROW_UNLESS(node["bounding_region"].IsDefined());
  MALIPUT_THROW_UNLESS(node["properties"].IsDefined());
  std::unique_ptr<maliput::math::BoundingBox> bounding_box = ParseBoundingBox(node["bounding_region"], tolerance);
  const std::map<std::string, std::string> properties = ParseProperties(node["properties"]);
  return std::make_unique<api::Object<maliput::math::Vector3>>(api::Object<maliput::math::Vector3>::Id(id), properties,
                                                               std::move(bounding_box));
}

std::unique_ptr<maliput::object::api::ObjectBook<maliput::math::Vector3>> BuildFrom(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node.IsMap());
  const YAML::Node& objects_node = node["maliput_objects"];
  MALIPUT_THROW_UNLESS(objects_node.IsDefined());
  MALIPUT_THROW_UNLESS(objects_node.IsMap());
  auto object_book = std::make_unique<ManualObjectBook<maliput::math::Vector3>>();
  for (const auto& object_node : objects_node) {
    object_book->AddObject(ParseObject(object_node.first.as<std::string>(), object_node.second, kTolerance));
  }
  return object_book;
}

}  // namespace

std::unique_ptr<maliput::object::api::ObjectBook<maliput::math::Vector3>> Load(const std::string& input) {
  return BuildFrom(YAML::Load(input));
}

std::unique_ptr<maliput::object::api::ObjectBook<maliput::math::Vector3>> LoadFile(const std::string& filename) {
  return BuildFrom(YAML::LoadFile(filename));
}

}  // namespace loader
}  // namespace object
}  // namespace maliput
