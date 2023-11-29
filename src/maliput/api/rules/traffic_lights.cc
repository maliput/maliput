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
  result.emplace(BulbType::kRound, "Round");
  result.emplace(BulbType::kArrow, "Arrow");
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
           const std::optional<std::vector<BulbState>>& states, BoundingBox bounding_box)
    : id_(id),
      position_bulb_group_(position_bulb_group),
      orientation_bulb_group_(orientation_bulb_group),
      color_(color),
      type_(type),
      arrow_orientation_rad_(arrow_orientation_rad),
      bounding_box_(std::move(bounding_box)) {
  MALIPUT_THROW_UNLESS(type_ != BulbType::kArrow || arrow_orientation_rad_ != std::nullopt);
  if (type_ != BulbType::kArrow) {
    MALIPUT_THROW_UNLESS(arrow_orientation_rad_ == std::nullopt);
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
  MALIPUT_THROW_UNLESS(bulb_group_ != nullptr);
  MALIPUT_THROW_UNLESS(bulb_group_->traffic_light() != nullptr);
  return UniqueBulbId(bulb_group_->traffic_light()->id(), bulb_group_->id(), id_);
}

BulbGroup::BulbGroup(const BulbGroup::Id& id, const InertialPosition& position_traffic_light,
                     const Rotation& orientation_traffic_light, std::vector<std::unique_ptr<Bulb>> bulbs)
    : id_(id),
      position_traffic_light_(position_traffic_light),
      orientation_traffic_light_(orientation_traffic_light),
      bulbs_(std::move(bulbs)) {
  MALIPUT_THROW_UNLESS(bulbs_.size() > 0);
  MALIPUT_THROW_UNLESS(std::find_if(bulbs_.begin(), bulbs_.end(), [](const auto& bulb) { return bulb == nullptr; }) ==
                       bulbs_.end());
  for (auto& bulb : bulbs_) {
    MALIPUT_THROW_UNLESS(std::count_if(bulbs_.begin(), bulbs_.end(),
                                       [bulb_id = bulb->id()](const auto& b) { return bulb_id == b->id(); }) == 1);
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
  MALIPUT_THROW_UNLESS(traffic_light_ != nullptr);
  return UniqueBulbGroupId(traffic_light_->id(), id_);
}

TrafficLight::TrafficLight(const TrafficLight::Id& id, const InertialPosition& position_road_network,
                           const Rotation& orientation_road_network,
                           std::vector<std::unique_ptr<BulbGroup>> bulb_groups)
    : id_(id),
      position_road_network_(position_road_network),
      orientation_road_network_(orientation_road_network),
      bulb_groups_(std::move(bulb_groups)) {
  MALIPUT_THROW_UNLESS(std::find_if(bulb_groups_.begin(), bulb_groups_.end(), [](const auto& bulb_group) {
                         return bulb_group == nullptr;
                       }) == bulb_groups_.end());
  for (auto& bulb_group : bulb_groups_) {
    MALIPUT_THROW_UNLESS(
        std::count_if(bulb_groups_.begin(), bulb_groups_.end(),
                      [bulb_group_id = bulb_group->id()](const auto& bg) { return bulb_group_id == bg->id(); }) == 1);
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

const std::string UniqueBulbId::delimiter() { return "-"; }

const std::string UniqueBulbGroupId::delimiter() { return "-"; }

}  // namespace rules
}  // namespace api
}  // namespace maliput
