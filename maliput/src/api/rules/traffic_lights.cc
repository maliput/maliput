#include "maliput/api/rules/traffic_lights.h"

#include <algorithm>
#include <utility>

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

std::unordered_map<BulbColor, const char*, drake::DefaultHash> BulbColorMapper() {
  std::unordered_map<BulbColor, const char*, drake::DefaultHash> result;
  result.emplace(BulbColor::kRed, "Red");
  result.emplace(BulbColor::kYellow, "Yellow");
  result.emplace(BulbColor::kGreen, "Green");
  return result;
}

std::unordered_map<BulbType, const char*, drake::DefaultHash> BulbTypeMapper() {
  std::unordered_map<BulbType, const char*, drake::DefaultHash> result;
  result.emplace(BulbType::kRound, "Round");
  result.emplace(BulbType::kArrow, "Arrow");
  return result;
}

std::unordered_map<BulbState, const char*, drake::DefaultHash> BulbStateMapper() {
  std::unordered_map<BulbState, const char*, drake::DefaultHash> result;
  result.emplace(BulbState::kOff, "Off");
  result.emplace(BulbState::kOn, "On");
  result.emplace(BulbState::kBlinking, "Blinking");
  return result;
}

Bulb::Bulb(const Bulb::Id& id, const GeoPosition& position_bulb_group, const Rotation& orientation_bulb_group,
           const BulbColor& color, const BulbType& type, const drake::optional<double>& arrow_orientation_rad,
           const drake::optional<std::vector<BulbState>>& states, BoundingBox bounding_box)
    : id_(id),
      position_bulb_group_(position_bulb_group),
      orientation_bulb_group_(orientation_bulb_group),
      color_(color),
      type_(type),
      arrow_orientation_rad_(arrow_orientation_rad),
      bounding_box_(std::move(bounding_box)) {
  MALIPUT_THROW_UNLESS(type_ != BulbType::kArrow || arrow_orientation_rad_ != drake::nullopt);
  if (type_ != BulbType::kArrow) {
    MALIPUT_THROW_UNLESS(arrow_orientation_rad_ == drake::nullopt);
  }
  if (states == drake::nullopt || states->size() == 0) {
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

GeoPosition Bulb::WorldFramePosition() const {
  MALIPUT_THROW_UNLESS(bulb_group_ != nullptr);

  return GeoPosition::FromXyz(bulb_group_->WorldFramePosition().xyz() +
                              bulb_group_->orientation_traffic_light().matrix() * position_bulb_group_.xyz());
}

BulbGroup::BulbGroup(const BulbGroup::Id& id, const GeoPosition& position_traffic_light,
                     const Rotation& orientation_traffic_light, const std::vector<Bulb>& bulbs)
    : id_(id),
      position_traffic_light_(position_traffic_light),
      orientation_traffic_light_(orientation_traffic_light),
      bulbs_(bulbs) {
  MALIPUT_THROW_UNLESS(bulbs_.size() > 0);
  for (const Bulb& bulb : bulbs_) {
    MALIPUT_THROW_UNLESS(std::count_if(bulbs_.begin(), bulbs_.end(),
                                       [bulb_id = bulb.id()](const Bulb& b) { return bulb_id == b.id(); }) == 1);
    BulbGroupAttorney::SetBulbGroupToBulb(this, const_cast<Bulb*>(&bulb));
  }
}

drake::optional<Bulb> BulbGroup::GetBulb(const Bulb::Id& id) const {
  for (const auto& bulb : bulbs_) {
    if (bulb.id() == id) {
      return bulb;
    }
  }
  return drake::nullopt;
}


GeoPosition BulbGroup::WorldFramePosition() const {
  MALIPUT_THROW_UNLESS(traffic_light_ != nullptr);

  return GeoPosition::FromXyz(
      traffic_light_->position_road_network().xyz() +
      traffic_light_->orientation_road_network().matrix() * position_traffic_light_.xyz());
}

TrafficLight::TrafficLight(const TrafficLight::Id& id, const GeoPosition& position_road_network,
                           const Rotation& orientation_road_network, const std::vector<BulbGroup>& bulb_groups)
    : id_(id),
      position_road_network_(position_road_network),
      orientation_road_network_(orientation_road_network),
      bulb_groups_(bulb_groups) {
  for (const BulbGroup bulb_group : bulb_groups_) {
    MALIPUT_THROW_UNLESS(
        std::count_if(bulb_groups_.begin(), bulb_groups_.end(), [bulb_group_id = bulb_group.id()](const BulbGroup& bg) {
          return bulb_group_id == bg.id();
        }) == 1);
    TrafficLightAttorney::SetTrafficLightToBulbGroup(this, const_cast<BulbGroup*>(&bulb_group));
  }
}

drake::optional<BulbGroup> TrafficLight::GetBulbGroup(const BulbGroup::Id& id) const {
  for (const auto& bulb_group : bulb_groups_) {
    if (bulb_group.id() == id) {
      return bulb_group;
    }
  }
  return drake::nullopt;
}

void BulbGroupAttorney::SetBulbGroupToBulb(const BulbGroup* bulb_group, Bulb* bulb) {
  bulb->set_bulb_group(bulb_group);
}

void TrafficLightAttorney::SetTrafficLightToBulbGroup(const TrafficLight* traffic_light, BulbGroup* bulb_group) {
  bulb_group->set_traffic_light(traffic_light);
}


}  // namespace rules
}  // namespace api
}  // namespace maliput
