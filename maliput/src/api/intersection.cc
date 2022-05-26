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
#include "maliput/api/intersection.h"

namespace maliput {
namespace api {

using rules::BulbStates;
using rules::PhaseProvider;
using rules::PhaseRing;

Intersection::Intersection(const Id& id, const std::vector<LaneSRange>& region, const PhaseRing& ring)
    : id_(id), region_(region), ring_(ring) {}

const std::optional<BulbStates> Intersection::bulb_states() const {
  const std::optional<PhaseProvider::Result> phase_result = Phase();
  if (phase_result.has_value()) {
    const rules::Phase::Id phase_id = phase_result->state;
    const rules::Phase& phase = ring_.phases().at(phase_id);
    return phase.bulb_states();
  }
  return std::nullopt;
}

const std::optional<rules::DiscreteValueRuleStates> Intersection::DiscreteValueRuleStates() const {
  const std::optional<PhaseProvider::Result> phase_result = Phase();
  if (phase_result.has_value()) {
    const rules::Phase::Id phase_id = phase_result->state;
    const rules::Phase& phase = ring_.phases().at(phase_id);
    return phase.discrete_value_rule_states();
  }
  return std::nullopt;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
const std::optional<rules::RuleStates> Intersection::RuleStates() const {
  const std::optional<PhaseProvider::Result> phase_result = Phase();
  if (phase_result.has_value()) {
    const rules::Phase::Id phase_id = phase_result->state;
    const rules::Phase& phase = ring_.phases().at(phase_id);
    const auto rule_states = phase.rule_states();
    return rule_states;
  }
  return std::nullopt;
}
#pragma GCC diagnostic pop

bool Intersection::Includes(const api::rules::TrafficLight::Id& id) const {
  const std::optional<api::rules::BulbStates> bulb_states = this->bulb_states();
  if (bulb_states.has_value()) {
    for (const auto& bulb_state : bulb_states.value()) {
      if (bulb_state.first.traffic_light_id() == id) {
        return true;
      }
    }
  }
  return false;
}

bool Intersection::Includes(const api::rules::DiscreteValueRule::Id& id) const {
  const std::optional<rules::DiscreteValueRuleStates> discrete_value_rule_states = DiscreteValueRuleStates();
  return discrete_value_rule_states.has_value()
             ? discrete_value_rule_states.value().find(id) != discrete_value_rule_states.value().end()
             : false;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
bool Intersection::Includes(const api::rules::RightOfWayRule::Id& id) const {
  const std::optional<rules::RuleStates> rule_states = RuleStates();
  return rule_states.has_value() ? rule_states.value().find(id) != rule_states.value().end() : false;
}
#pragma GCC diagnostic pop

bool Intersection::Includes(const InertialPosition& inertial_position, const RoadGeometry* road_geometry) const {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  return IsIncluded(inertial_position, region_, road_geometry);
}

}  // namespace api
}  // namespace maliput
