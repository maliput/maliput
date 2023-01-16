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
#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "maliput/api/intersection_book.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/range_value_rule_state_provider.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule_registry.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/api/rules/traffic_light_book.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {

/// A container that aggregates everything pertaining to Maliput.
class RoadNetwork {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetwork)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Constructs a RoadNetwork instance. After creation, you are encouraged to
  /// validate it using ValidateRoadNetwork(), which is defined in
  /// maliput/api/road_network_validator.h.
  /// TODO(#108): Remove this constructor once old rule api is removed.
  RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry, std::unique_ptr<const rules::RoadRulebook> rulebook,
              std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
              std::unique_ptr<IntersectionBook> intersection_book,
              std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
              std::unique_ptr<rules::RightOfWayRuleStateProvider> right_of_way_rule_state_provider,
              std::unique_ptr<rules::PhaseProvider> phase_provider, std::unique_ptr<rules::RuleRegistry> rule_registry,
              std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider,
              std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider);
#pragma GCC diagnostic pop

  /// Constructs a RoadNetwork instance. After creation, you are encouraged to
  /// validate it using ValidateRoadNetwork(), which is defined in
  /// maliput/api/road_network_validator.h.
  RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry, std::unique_ptr<const rules::RoadRulebook> rulebook,
              std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
              std::unique_ptr<IntersectionBook> intersection_book,
              std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
              std::unique_ptr<rules::PhaseProvider> phase_provider, std::unique_ptr<rules::RuleRegistry> rule_registry,
              std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider,
              std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider);

  virtual ~RoadNetwork() = default;

  /// Determines if the road network contains @p road_position
  bool Contains(const RoadPosition& road_position) const;

  /// Determines if the road networks contains @p lane_id
  bool Contains(const LaneId& lane_id) const;

  const RoadGeometry* road_geometry() const { return road_geometry_.get(); }

  const rules::RoadRulebook* rulebook() const { return rulebook_.get(); }

  const rules::TrafficLightBook* traffic_light_book() const { return traffic_light_book_.get(); }

  IntersectionBook* intersection_book() { return intersection_book_.get(); }

  const rules::PhaseRingBook* phase_ring_book() const { return phase_ring_book_.get(); }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MALIPUT_DEPRECATED("RightOfWayRule class will be deprecated.")
  rules::RightOfWayRuleStateProvider* right_of_way_rule_state_provider() {
    return right_of_way_rule_state_provider_.get();
  }
#pragma GCC diagnostic pop

  rules::PhaseProvider* phase_provider() { return phase_provider_.get(); }

  const rules::RuleRegistry* rule_registry() const { return rule_registry_.get(); }

  rules::DiscreteValueRuleStateProvider* discrete_value_rule_state_provider() {
    return discrete_value_rule_state_provider_.get();
  }

  rules::RangeValueRuleStateProvider* range_value_rule_state_provider() {
    return range_value_rule_state_provider_.get();
  }

 private:
  std::unique_ptr<const RoadGeometry> road_geometry_;
  std::unique_ptr<const rules::RoadRulebook> rulebook_;
  std::unique_ptr<const rules::TrafficLightBook> traffic_light_book_;
  std::unique_ptr<IntersectionBook> intersection_book_;
  std::unique_ptr<rules::PhaseRingBook> phase_ring_book_;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::unique_ptr<rules::RightOfWayRuleStateProvider> right_of_way_rule_state_provider_;
#pragma GCC diagnostic pop
  std::unique_ptr<rules::PhaseProvider> phase_provider_;
  std::unique_ptr<rules::RuleRegistry> rule_registry_;
  std::unique_ptr<rules::DiscreteValueRuleStateProvider> discrete_value_rule_state_provider_;
  std::unique_ptr<rules::RangeValueRuleStateProvider> range_value_rule_state_provider_;
};

}  // namespace api
}  // namespace maliput
