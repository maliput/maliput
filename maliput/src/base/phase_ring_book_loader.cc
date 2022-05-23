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
#include "maliput/base/phase_ring_book_loader.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/base/manual_phase_ring_book.h"
#include "maliput/base/rule_registry.h"
#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"

namespace YAML {

template <>
struct convert<maliput::api::rules::BulbState> {
  static Node encode(const maliput::api::rules::BulbState& rhs) {
    Node node;
    node.push_back(maliput::api::rules::BulbStateMapper().at(rhs));
    return node;
  }

  // This API is required by yaml-cpp. See this web page for more information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  static bool decode(const Node& node,
                     // NOLINTNEXTLINE(runtime/references).
                     maliput::api::rules::BulbState& rhs) {
    const std::string color = node.as<std::string>();
    bool result = false;
    for (const auto& it : maliput::api::rules::BulbStateMapper()) {
      if (it.second == color) {
        rhs = it.first;
        result = true;
      }
    }
    return result;
  }
};

}  // namespace YAML

namespace maliput {
namespace {

using api::LaneSRange;
using api::rules::Bulb;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::BulbStates;
using api::rules::DiscreteValueRule;
using api::rules::DiscreteValueRuleStates;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RoadRulebook;
using api::rules::Rule;
using api::rules::TrafficLight;
using api::rules::TrafficLightBook;
using api::rules::UniqueBulbGroupId;
using api::rules::UniqueBulbId;

// Given @p rulebook that contains all of the rules, and @p rules_node that
// contains a sequence of rule IDs, return a
// std::unordered_map<Rule::Id, DiscreteValueRule> of the rules mentioned
// in @p rules_node.
std::unordered_map<Rule::Id, DiscreteValueRule> GetRightOfWayTypeRules(const RoadRulebook* rulebook,
                                                                       const YAML::Node& rules_node) {
  MALIPUT_THROW_UNLESS(rules_node.IsSequence());
  std::unordered_map<Rule::Id, DiscreteValueRule> result;
  for (const YAML::Node& rule_node : rules_node) {
    const Rule::Id rule_id(rule_node.as<std::string>());
    result.emplace(rule_id, rulebook->GetDiscreteValueRule(rule_id));
  }
  return result;
}

DiscreteValueRule::DiscreteValue GetDefaultState(const std::vector<DiscreteValueRule::DiscreteValue>& values) {
  for (const auto& keyword : {"Stop", "StopThenGo", "Go"}) {
    auto it = std::find_if(values.begin(), values.end(), [&keyword](const auto& v) { return v.value == keyword; });
    if (it != values.end()) {
      return *it;
    }
  }
  MALIPUT_ABORT_MESSAGE("The rule has no states.");
}

// Given a set of rules, determine default states for each rule and return them
// in a new RuleStates object. See GetDefaultState() for details on how the
// default state is determined.
DiscreteValueRuleStates CreateDefaultRuleStates(const std::unordered_map<Rule::Id, DiscreteValueRule> rules) {
  DiscreteValueRuleStates result;
  for (const auto& rule : rules) {
    result.emplace(rule.first, GetDefaultState(rule.second.states()));
  }
  return result;
}

// Confirms that every bulb in @p bulbs_node exists within @p bulb_group.
void ConfirmBulbsExist(const BulbGroup& bulb_group, const YAML::Node& bulbs_node) {
  for (const auto& bulb_group_pair : bulbs_node) {
    const Bulb::Id bulb_id(bulb_group_pair.first.as<std::string>());
    MALIPUT_THROW_UNLESS(bulb_group.GetBulb(bulb_id) != nullptr);
  }
}

std::optional<BulbStates> LoadBulbStates(const TrafficLightBook* traffic_light_book, const YAML::Node& phase_node) {
  std::optional<BulbStates> result;
  const YAML::Node& traffic_light_states_node = phase_node["TrafficLightStates"];
  if (traffic_light_states_node.IsDefined()) {
    MALIPUT_THROW_UNLESS(traffic_light_states_node.IsMap());
    result = BulbStates();
    for (const auto& traffic_light_pair : traffic_light_states_node) {
      const TrafficLight::Id traffic_light_id(traffic_light_pair.first.as<std::string>());
      const TrafficLight* traffic_light = traffic_light_book->GetTrafficLight(traffic_light_id);
      MALIPUT_THROW_UNLESS(traffic_light != nullptr);
      const YAML::Node& bulb_group_node = traffic_light_pair.second;
      MALIPUT_THROW_UNLESS(bulb_group_node.IsDefined());
      MALIPUT_THROW_UNLESS(bulb_group_node.IsMap());
      for (const auto& bulb_group_pair : bulb_group_node) {
        const BulbGroup::Id bulb_group_id(bulb_group_pair.first.as<std::string>());
        const BulbGroup* bulb_group = traffic_light->GetBulbGroup(bulb_group_id);
        MALIPUT_THROW_UNLESS(bulb_group != nullptr);
        const YAML::Node& bulbs_node = bulb_group_pair.second;
        MALIPUT_THROW_UNLESS(bulbs_node.IsDefined());
        MALIPUT_THROW_UNLESS(bulbs_node.IsMap());
        ConfirmBulbsExist(*bulb_group, bulbs_node);
        for (const Bulb* bulb : bulb_group->bulbs()) {
          BulbState bulb_state = bulb->GetDefaultState();
          const YAML::Node& bulb_state_node = bulbs_node[bulb->id().string()];
          if (bulb_state_node.IsDefined()) {
            bulb_state = bulb_state_node.as<BulbState>();
          }
          (*result)[bulb->unique_id()] = bulb_state;
        }
      }
    }
  }
  return result;
}

void VerifyPhaseExists(const std::vector<Phase>& phases, const Phase::Id& phase_id) {
  const auto it =
      std::find_if(phases.begin(), phases.end(), [&](const Phase& p) -> bool { return p.id() == phase_id; });
  MALIPUT_THROW_UNLESS(it != phases.end());
}

std::optional<const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>>> BuildNextPhases(
    const std::vector<Phase>& phases, const YAML::Node& phase_ring_node) {
  const YAML::Node& graph_node = phase_ring_node["PhaseTransitionGraph"];
  if (!graph_node.IsDefined()) {
    return std::nullopt;
  }
  std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>> result;
  MALIPUT_THROW_UNLESS(phase_ring_node.IsMap());
  for (const auto& graph_node_it : graph_node) {
    const Phase::Id phase_id(graph_node_it.first.as<std::string>());
    VerifyPhaseExists(phases, phase_id);
    const YAML::Node& next_phases_node = graph_node_it.second;
    MALIPUT_THROW_UNLESS(next_phases_node.IsSequence());
    std::vector<PhaseRing::NextPhase> next_phases;
    for (const YAML::Node& next_phase_node : next_phases_node) {
      MALIPUT_THROW_UNLESS(next_phase_node.IsMap());
      MALIPUT_THROW_UNLESS(next_phase_node["ID"].IsDefined());
      const Phase::Id next_phase_id(next_phase_node["ID"].as<std::string>());
      VerifyPhaseExists(phases, next_phase_id);
      std::optional<double> duration_until = std::nullopt;
      if (next_phase_node["duration_until"].IsDefined()) {
        duration_until = next_phase_node["duration_until"].as<double>();
      }
      next_phases.push_back(PhaseRing::NextPhase{next_phase_id, duration_until});
    }
    result.emplace(phase_id, next_phases);
  }

  return result;
}

PhaseRing BuildPhaseRing(const RoadRulebook* rulebook, const TrafficLightBook* traffic_light_book,
                         const YAML::Node& phase_ring_node) {
  MALIPUT_THROW_UNLESS(phase_ring_node.IsMap());
  MALIPUT_THROW_UNLESS(phase_ring_node["ID"].IsDefined());
  const PhaseRing::Id ring_id(phase_ring_node["ID"].as<std::string>());

  const std::unordered_map<Rule::Id, DiscreteValueRule> discrete_rules =
      GetRightOfWayTypeRules(rulebook, phase_ring_node["Rules"]);
  MALIPUT_THROW_UNLESS(!discrete_rules.empty());
  // First get default states of all rules.
  // Then, override the defaults with the states specified in the YAML
  // document.
  DiscreteValueRuleStates discrete_rule_states = CreateDefaultRuleStates(discrete_rules);
  MALIPUT_THROW_UNLESS(!discrete_rule_states.empty());

  const YAML::Node& phases_node = phase_ring_node["Phases"];
  MALIPUT_THROW_UNLESS(phases_node.IsDefined());
  MALIPUT_THROW_UNLESS(phases_node.IsSequence());
  std::vector<Phase> phases;
  for (const YAML::Node& phase_node : phases_node) {
    MALIPUT_THROW_UNLESS(phase_node.IsMap());
    MALIPUT_THROW_UNLESS(phase_node["ID"].IsDefined());
    const Phase::Id phase_id(phase_node["ID"].as<std::string>());
    const YAML::Node& rule_states_node = phase_node["RightOfWayRuleStates"];
    MALIPUT_THROW_UNLESS(rule_states_node.IsDefined());
    MALIPUT_THROW_UNLESS(rule_states_node.IsMap());
    for (const auto& rule_state_it : rule_states_node) {
      Rule::Id rule_id(rule_state_it.first.as<std::string>());
      std::string value(rule_state_it.second.as<std::string>());
      MALIPUT_THROW_UNLESS(discrete_rules.find(rule_id) != discrete_rules.end());
      const auto discrete_value_itr = std::find_if(
          discrete_rules.at(rule_id).states().begin(), discrete_rules.at(rule_id).states().end(),
          [&value](const DiscreteValueRule::DiscreteValue& discrete_value) { return discrete_value.value == value; });
      MALIPUT_THROW_UNLESS(discrete_value_itr != discrete_rules.at(rule_id).states().end());
      MALIPUT_THROW_UNLESS(discrete_rule_states.find(rule_id) != discrete_rule_states.end());
      discrete_rule_states.at(rule_id) = *discrete_value_itr;
    }
    phases.push_back(Phase(phase_id, {}, discrete_rule_states, LoadBulbStates(traffic_light_book, phase_node)));
  }

  const auto next_phases = BuildNextPhases(phases, phase_ring_node);
  return PhaseRing(ring_id, phases, next_phases);
}

std::unique_ptr<api::rules::PhaseRingBook> BuildFrom(const RoadRulebook* rulebook,
                                                     const TrafficLightBook* traffic_light_book,
                                                     const YAML::Node& root_node) {
  MALIPUT_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& phase_rings_node = root_node["PhaseRings"];
  MALIPUT_THROW_UNLESS(phase_rings_node.IsDefined());
  MALIPUT_THROW_UNLESS(phase_rings_node.IsSequence());
  auto result = std::make_unique<ManualPhaseRingBook>();
  for (const YAML::Node& phase_ring_node : phase_rings_node) {
    result->AddPhaseRing(BuildPhaseRing(rulebook, traffic_light_book, phase_ring_node));
  }
  return result;
}

}  // namespace

std::unique_ptr<api::rules::PhaseRingBook> LoadPhaseRingBook(const RoadRulebook* rulebook,
                                                             const TrafficLightBook* traffic_light_book,
                                                             const std::string& input) {
  return BuildFrom(rulebook, traffic_light_book, YAML::Load(input));
}

std::unique_ptr<api::rules::PhaseRingBook> LoadPhaseRingBookFromFile(const RoadRulebook* rulebook,
                                                                     const TrafficLightBook* traffic_light_book,
                                                                     const std::string& filename) {
  return BuildFrom(rulebook, traffic_light_book, YAML::LoadFile(filename));
}

}  // namespace maliput
