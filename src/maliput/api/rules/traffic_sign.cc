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
#include "maliput/api/rules/traffic_sign.h"

#include <utility>

namespace maliput {
namespace api {
namespace rules {

std::unordered_map<TrafficSignType, const char*, maliput::common::DefaultHash> TrafficSignTypeMapper() {
  std::unordered_map<TrafficSignType, const char*, maliput::common::DefaultHash> result;
  result.emplace(TrafficSignType::kStop, "Stop");
  result.emplace(TrafficSignType::kYield, "Yield");
  result.emplace(TrafficSignType::kSpeedLimit, "SpeedLimit");
  result.emplace(TrafficSignType::kNoEntry, "NoEntry");
  result.emplace(TrafficSignType::kOneWay, "OneWay");
  result.emplace(TrafficSignType::kPedestrianCrossing, "PedestrianCrossing");
  result.emplace(TrafficSignType::kNoLeftTurn, "NoLeftTurn");
  result.emplace(TrafficSignType::kNoRightTurn, "NoRightTurn");
  result.emplace(TrafficSignType::kNoUTurn, "NoUTurn");
  result.emplace(TrafficSignType::kSchoolZone, "SchoolZone");
  result.emplace(TrafficSignType::kConstruction, "Construction");
  result.emplace(TrafficSignType::kRailroadCrossing, "RailroadCrossing");
  result.emplace(TrafficSignType::kNoOvertaking, "NoOvertaking");
  result.emplace(TrafficSignType::kUnknown, "Unknown");
  return result;
}

TrafficSign::TrafficSign(const Id& id, const TrafficSignType& type, const InertialPosition& position_road_network,
                         const Rotation& orientation_road_network, const std::optional<std::string>& message,
                         std::vector<LaneId> related_lanes, const maliput::math::BoundingBox& bounding_box)
    : id_(id),
      type_(type),
      position_road_network_(position_road_network),
      orientation_road_network_(orientation_road_network),
      message_(message),
      related_lanes_(std::move(related_lanes)),
      bounding_box_(bounding_box) {}

}  // namespace rules
}  // namespace api
}  // namespace maliput
