// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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
#include "maliput/api/rules/traffic_lights.h"

#include <algorithm>
#include <utility>

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

std::unordered_map<BulbColor, const char*, maliput::common::DefaultHash> BulbColorMapper() {
  std::unordered_map<BulbColor, const char*, maliput::common::DefaultHash> result;
  result.emplace(BulbColor::kRed, "Red");
  result.emplace(BulbColor::kYellow, "Yellow");
  result.emplace(BulbColor::kGreen, "Green");
  return result;
}

std::unordered_map<BulbType, const char*, maliput::common::DefaultHash> BulbTypeMapper() {
  std::unordered_map<BulbType, const char*, maliput::common::DefaultHash> result;
  result.emplace(BulbType::kRound, "round");
  result.emplace(BulbType::kArrow, "arrow");
  result.emplace(BulbType::kArrowLeft, "arrow_left");
  result.emplace(BulbType::kArrowRight, "arrow_right");
  result.emplace(BulbType::kArrowUp, "arrow_up");
  result.emplace(BulbType::kArrowUpperLeft, "arrow_upper_left");
  result.emplace(BulbType::kArrowUpperRight, "arrow_upper_right");
  result.emplace(BulbType::kUTurnLeft, "u_turn_left");
  result.emplace(BulbType::kUTurnRight, "u_turn_right");
  result.emplace(BulbType::kWalk, "walk");
  result.emplace(BulbType::kDontWalk, "dont_walk");
  result.emplace(BulbType::kCross, "cross");
  result.emplace(BulbType::kPedestrian, "pedestrian");
  result.emplace(BulbType::kBicycle, "bicycle");
  result.emplace(BulbType::kPedestrianAndBicycle, "pedestrian_and_bicycle");
  result.emplace(BulbType::kTram, "tram");
  result.emplace(BulbType::kBus, "bus");
  result.emplace(BulbType::kBusAndTram, "bus_and_tram");
  result.emplace(BulbType::kCountdownInSeconds, "countdown_in_seconds");
  result.emplace(BulbType::kCountdownInPercent, "countdown_in_percent");
  return result;
}

std::unordered_map<std::string, BulbType> BulbTypeStringToEnumMapper() {
  std::unordered_map<std::string, BulbType> result;
  result.emplace("round", BulbType::kRound);
  result.emplace("Round", BulbType::kRound);
  result.emplace("arrow", BulbType::kArrow);
  result.emplace("Arrow", BulbType::kArrow);
  result.emplace("arrow_left", BulbType::kArrowLeft);
  result.emplace("ArrowLeft", BulbType::kArrowLeft);
  result.emplace("arrow_right", BulbType::kArrowRight);
  result.emplace("ArrowRight", BulbType::kArrowRight);
  result.emplace("arrow_up", BulbType::kArrowUp);
  result.emplace("ArrowUp", BulbType::kArrowUp);
  result.emplace("arrow_upper_left", BulbType::kArrowUpperLeft);
  result.emplace("ArrowUpperLeft", BulbType::kArrowUpperLeft);
  result.emplace("arrow_upper_right", BulbType::kArrowUpperRight);
  result.emplace("ArrowUpperRight", BulbType::kArrowUpperRight);
  result.emplace("u_turn_left", BulbType::kUTurnLeft);
  result.emplace("UTurnLeft", BulbType::kUTurnLeft);
  result.emplace("u_turn_right", BulbType::kUTurnRight);
  result.emplace("UTurnRight", BulbType::kUTurnRight);
  result.emplace("walk", BulbType::kWalk);
  result.emplace("Walk", BulbType::kWalk);
  result.emplace("dont_walk", BulbType::kDontWalk);
  result.emplace("DontWalk", BulbType::kDontWalk);
  result.emplace("cross", BulbType::kCross);
  result.emplace("Cross", BulbType::kCross);
  result.emplace("pedestrian", BulbType::kPedestrian);
  result.emplace("Pedestrian", BulbType::kPedestrian);
  result.emplace("bicycle", BulbType::kBicycle);
  result.emplace("Bicycle", BulbType::kBicycle);
  result.emplace("pedestrian_and_bicycle", BulbType::kPedestrianAndBicycle);
  result.emplace("PedestrianAndBicycle", BulbType::kPedestrianAndBicycle);
  result.emplace("tram", BulbType::kTram);
  result.emplace("Tram", BulbType::kTram);
  result.emplace("bus", BulbType::kBus);
  result.emplace("Bus", BulbType::kBus);
  result.emplace("bus_and_tram", BulbType::kBusAndTram);
  result.emplace("BusAndTram", BulbType::kBusAndTram);
  result.emplace("countdown_in_seconds", BulbType::kCountdownInSeconds);
  result.emplace("CountdownInSeconds", BulbType::kCountdownInSeconds);
  result.emplace("countdown_in_percent", BulbType::kCountdownInPercent);
  result.emplace("CountdownInPercent", BulbType::kCountdownInPercent);
  return result;
}

std::unordered_map<BulbState, const char*, maliput::common::DefaultHash> BulbStateMapper() {
  std::unordered_map<BulbState, const char*, maliput::common::DefaultHash> result;
  result.emplace(BulbState::kOff, "Off");
  result.emplace(BulbState::kOn, "On");
  result.emplace(BulbState::kBlinking, "Blinking");
  return result;
}

Bulb::Bulb(const Bulb::Id& id, const InertialPosition& position_bulb_group, const Rotation& orientation_bulb_group,
           const BulbColor& color, const BulbType& type, const std::optional<double>& arrow_orientation_rad,
           const std::optional<std::vector<BulbState>>& states, BoundingBox bounding_box,
           const std::optional<BulbState>& initial_state)
    : id_(id),
      position_bulb_group_(position_bulb_group),
      orientation_bulb_group_(orientation_bulb_group),
      color_(color),
      type_(type),
      arrow_orientation_rad_(arrow_orientation_rad),
      initial_state_(initial_state.value_or(BulbState::kOff)),
      bounding_box_(std::move(bounding_box)) {
  MALIPUT_VALIDATE(type_ != BulbType::kArrow || arrow_orientation_rad_ != std::nullopt,
                   "Arrow-typed bulb's orientation is null.", maliput::common::traffic_light_book_error);
  if (type_ != BulbType::kArrow) {
    MALIPUT_VALIDATE(arrow_orientation_rad_ == std::nullopt, "Non-arrow-typed bulb cannot have an orientation.",
                     maliput::common::traffic_light_book_error);
  }
  if (states == std::nullopt || states->size() == 0) {
    states_ = {BulbState::kOff, BulbState::kOn};
  } else {
    states_ = *states;
  }
}

BulbState Bulb::GetDefaultState() const {
  for (const auto& bulb_state : {BulbState::kOff, BulbState::kBlinking, BulbState::kOn}) {
    if (IsValidState(bulb_state)) {
      return bulb_state;
    }
  }
  MALIPUT_ABORT_MESSAGE("bulb_state is not valid.");
}

bool Bulb::IsValidState(const BulbState& bulb_state) const {
  return std::find(states_.begin(), states_.end(), bulb_state) != states_.end();
}

UniqueBulbId Bulb::unique_id() const {
  MALIPUT_VALIDATE(bulb_group_ != nullptr, "BulbGroup is null.", maliput::common::traffic_light_book_error);
  MALIPUT_VALIDATE(bulb_group_->traffic_light() != nullptr, "BulbGroup's traffic light is null.",
                   maliput::common::traffic_light_book_error);
  return UniqueBulbId(bulb_group_->traffic_light()->id(), bulb_group_->id(), id_);
}

BulbGroup::BulbGroup(const BulbGroup::Id& id, const InertialPosition& position_traffic_light,
                     const Rotation& orientation_traffic_light, std::vector<std::unique_ptr<Bulb>> bulbs)
    : id_(id),
      position_traffic_light_(position_traffic_light),
      orientation_traffic_light_(orientation_traffic_light),
      bulbs_(std::move(bulbs)) {
  MALIPUT_VALIDATE(bulbs_.size() > 0, "BulbGroup should have at least 1 bulb.",
                   maliput::common::traffic_light_book_error);
  MALIPUT_VALIDATE(
      std::find_if(bulbs_.begin(), bulbs_.end(), [](const auto& bulb) { return bulb == nullptr; }) == bulbs_.end(),
      "Found nulled bulb when creating BulbGroup.", maliput::common::traffic_light_book_error);
  for (auto& bulb : bulbs_) {
    MALIPUT_VALIDATE(std::count_if(bulbs_.begin(), bulbs_.end(),
                                   [bulb_id = bulb->id()](const auto& b) { return bulb_id == b->id(); }) == 1,
                     "Bulb with ID '" + bulb->id().string() + "' is not unique when creating BulbGroup.",
                     maliput::common::traffic_light_book_error);
    bulb->SetBulbGroup({}, this);
  }
}

const Bulb* BulbGroup::GetBulb(const Bulb::Id& id) const {
  for (const auto& bulb : bulbs_) {
    if (bulb->id() == id) {
      return bulb.get();
    }
  }
  return nullptr;
}

std::vector<const Bulb*> BulbGroup::bulbs() const {
  std::vector<const Bulb*> bulb_ptrs;
  for (const auto& bulb : bulbs_) {
    bulb_ptrs.emplace_back(bulb.get());
  }
  return bulb_ptrs;
}

UniqueBulbGroupId BulbGroup::unique_id() const {
  MALIPUT_VALIDATE(traffic_light_ != nullptr, "Traffic light is null.", maliput::common::traffic_light_book_error);
  return UniqueBulbGroupId(traffic_light_->id(), id_);
}

TrafficLight::TrafficLight(const TrafficLight::Id& id, const InertialPosition& position_road_network,
                           const Rotation& orientation_road_network,
                           std::vector<std::unique_ptr<BulbGroup>> bulb_groups, std::vector<LaneId> related_lanes)
    : id_(id),
      position_road_network_(position_road_network),
      orientation_road_network_(orientation_road_network),
      bulb_groups_(std::move(bulb_groups)),
      related_lanes_(std::move(related_lanes)) {
  MALIPUT_VALIDATE(std::find_if(bulb_groups_.begin(), bulb_groups_.end(),
                                [](const auto& bulb_group) { return bulb_group == nullptr; }) == bulb_groups_.end(),
                   "Found nulled bulb when creating TrafficLight.", maliput::common::traffic_light_book_error);
  for (auto& bulb_group : bulb_groups_) {
    MALIPUT_VALIDATE(
        std::count_if(bulb_groups_.begin(), bulb_groups_.end(),
                      [bulb_group_id = bulb_group->id()](const auto& bg) { return bulb_group_id == bg->id(); }) == 1,
        "BulbGroup with ID '" + bulb_group->id().string() + "' is not unique when creating TrafficLight.",
        maliput::common::traffic_light_book_error);
    bulb_group->SetTrafficLight({}, this);
  }
}

std::vector<const BulbGroup*> TrafficLight::bulb_groups() const {
  std::vector<const BulbGroup*> bulb_group_ptrs;
  for (const auto& bulb_group : bulb_groups_) {
    bulb_group_ptrs.emplace_back(bulb_group.get());
  }
  return bulb_group_ptrs;
}

const BulbGroup* TrafficLight::GetBulbGroup(const BulbGroup::Id& id) const {
  for (const auto& bulb_group : bulb_groups_) {
    if (bulb_group->id() == id) {
      return bulb_group.get();
    }
  }
  return nullptr;
}

std::map<UniqueBulbId, BulbState> TrafficLight::InitialBulbStates() const {
  std::map<UniqueBulbId, BulbState> result;
  for (const auto& bulb_group : bulb_groups_) {
    for (const auto* bulb : bulb_group->bulbs()) {
      result.emplace(UniqueBulbId(id_, bulb_group->id(), bulb->id()), bulb->GetInitialState());
    }
  }
  return result;
}

const std::string UniqueBulbId::delimiter() { return "-"; }

const std::string UniqueBulbGroupId::delimiter() { return "-"; }

}  // namespace rules
}  // namespace api
}  // namespace maliput
