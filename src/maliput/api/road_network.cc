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
#include "maliput/api/road_network.h"

#include <utility>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
RoadNetwork::RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry,
                         std::unique_ptr<const rules::RoadRulebook> rulebook,
                         std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
                         std::unique_ptr<IntersectionBook> intersection_book,
                         std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
                         std::unique_ptr<rules::RightOfWayRuleStateProvider> right_of_way_rule_state_provider,
                         std::unique_ptr<rules::PhaseProvider> phase_provider,
                         std::unique_ptr<rules::RuleRegistry> rule_registry,
                         std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider,
                         std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider)
    : RoadNetwork(std::move(road_geometry), std::move(rulebook), std::move(traffic_light_book),
                  std::move(intersection_book), std::move(phase_ring_book), std::move(phase_provider),
                  std::move(rule_registry), std::move(discrete_value_rule_state_provider),
                  std::move(range_value_rule_state_provider)) {
  right_of_way_rule_state_provider_ = std::move(right_of_way_rule_state_provider);
  MALIPUT_THROW_UNLESS(right_of_way_rule_state_provider_.get() != nullptr);
}

#pragma GCC diagnostic pop

RoadNetwork::RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry,
                         std::unique_ptr<const rules::RoadRulebook> rulebook,
                         std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
                         std::unique_ptr<IntersectionBook> intersection_book,
                         std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
                         std::unique_ptr<rules::PhaseProvider> phase_provider,
                         std::unique_ptr<rules::RuleRegistry> rule_registry,
                         std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider,
                         std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider)
    : road_geometry_(std::move(road_geometry)),
      rulebook_(std::move(rulebook)),
      traffic_light_book_(std::move(traffic_light_book)),
      intersection_book_(std::move(intersection_book)),
      phase_ring_book_(std::move(phase_ring_book)),
      phase_provider_(std::move(phase_provider)),
      rule_registry_(std::move(rule_registry)),
      discrete_value_rule_state_provider_(std::move(discrete_value_rule_state_provider)),
      range_value_rule_state_provider_(std::move(range_value_rule_state_provider)) {
  MALIPUT_THROW_UNLESS(road_geometry_.get() != nullptr);
  MALIPUT_THROW_UNLESS(rulebook_.get() != nullptr);
  MALIPUT_THROW_UNLESS(traffic_light_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(intersection_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(phase_ring_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(phase_provider_.get() != nullptr);
  MALIPUT_THROW_UNLESS(rule_registry_.get() != nullptr);
  MALIPUT_THROW_UNLESS(discrete_value_rule_state_provider_.get() != nullptr);
  MALIPUT_THROW_UNLESS(range_value_rule_state_provider_.get() != nullptr);
}

bool RoadNetwork::Contains(const RoadPosition& road_position) const {
  return road_position.lane->Contains(road_position.pos);
}

bool RoadNetwork::Contains(const LaneId& lane_id) const {
  return (this->road_geometry()->ById().GetLane(lane_id) != nullptr);
}

}  // namespace api
}  // namespace maliput
