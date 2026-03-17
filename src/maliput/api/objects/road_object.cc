// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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
#include "maliput/api/objects/road_object.h"

#include <utility>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace objects {

std::unordered_map<RoadObjectType, const char*, maliput::common::DefaultHash> RoadObjectTypeMapper() {
  return {
      {RoadObjectType::kUnknown, "Unknown"},
      {RoadObjectType::kBarrier, "Barrier"},
      {RoadObjectType::kBuilding, "Building"},
      {RoadObjectType::kCrosswalk, "Crosswalk"},
      {RoadObjectType::kGantry, "Gantry"},
      {RoadObjectType::kObstacle, "Obstacle"},
      {RoadObjectType::kParkingSpace, "ParkingSpace"},
      {RoadObjectType::kPole, "Pole"},
      {RoadObjectType::kRoadMark, "RoadMark"},
      {RoadObjectType::kRoadSurface, "RoadSurface"},
      {RoadObjectType::kTrafficIsland, "TrafficIsland"},
      {RoadObjectType::kTree, "Tree"},
      {RoadObjectType::kVegetation, "Vegetation"},
  };
}

Outline::Outline(const Id& id, std::vector<OutlineCorner> corners, bool closed)
    : id_(id), corners_(std::move(corners)), closed_(closed) {
  MALIPUT_VALIDATE(static_cast<int>(corners_.size()) >= 3,
                   "Outline '" + id_.string() + "' must have at least 3 corners.");
}

RoadObjectPosition::RoadObjectPosition(const InertialPosition& inertial_position)
    : inertial_position_(inertial_position) {}

RoadObjectPosition::RoadObjectPosition(const InertialPosition& inertial_position, const LaneId& lane_id,
                                       const LanePosition& lane_position)
    : inertial_position_(inertial_position), lane_id_(lane_id), lane_position_(lane_position) {}

RoadObject::RoadObject(const Id& id, RoadObjectType type, const RoadObjectPosition& position,
                       const Rotation& orientation, const maliput::math::BoundingBox& bounding_box, bool is_dynamic,
                       std::vector<LaneId> related_lanes, std::optional<std::string> name,
                       std::optional<std::string> subtype, std::vector<std::unique_ptr<Outline>> outlines,
                       std::unordered_map<std::string, std::string> properties)
    : id_(id),
      name_(std::move(name)),
      type_(type),
      subtype_(std::move(subtype)),
      position_(position),
      orientation_(orientation),
      bounding_box_(bounding_box),
      is_dynamic_(is_dynamic),
      related_lanes_(std::move(related_lanes)),
      outlines_(std::move(outlines)),
      properties_(std::move(properties)) {}

const Outline* RoadObject::outline(int index) const {
  MALIPUT_VALIDATE(
      index >= 0 && index < num_outlines(),
      "Outline index " + std::to_string(index) + " out of range [0, " + std::to_string(num_outlines()) + ").");
  return outlines_[index].get();
}

}  // namespace objects
}  // namespace api
}  // namespace maliput
