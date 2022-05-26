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
#include "maliput/base/intersection_book_loader.h"

#include <optional>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/base/intersection.h"
#include "maliput/base/intersection_book.h"
#include "maliput/common/maliput_throw.h"

using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;
using maliput::api::rules::PhaseRingBook;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RoadRulebook;
using maliput::api::rules::Rule;

namespace maliput {
namespace {

// TODO(liang.fok) Eliminate duplicate regions within the returned vector.
std::vector<LaneSRange> GetRegionOfOldRules(const RoadRulebook& road_rulebook, const Phase& phase) {
  std::vector<LaneSRange> result;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  for (const auto& rule_state : phase.rule_states()) {
    const RightOfWayRule::Id rule_id = rule_state.first;
    const RightOfWayRule rule = road_rulebook.GetRule(rule_id);
#pragma GCC diagnostic pop
    for (const auto& range : rule.zone().ranges()) {
      result.push_back(range);
    }
  }
  return result;
}

std::vector<LaneSRange> GetRegion(const RoadRulebook& road_rulebook, const Phase& phase) {
  std::vector<LaneSRange> result;
  for (const auto& rule_state : phase.discrete_value_rule_states()) {
    const Rule::Id rule_id = rule_state.first;
    const DiscreteValueRule rule = road_rulebook.GetDiscreteValueRule(rule_id);
    for (const auto& range : rule.zone().ranges()) {
      result.push_back(range);
    }
  }
  return result;
}

std::unique_ptr<api::Intersection> BuildIntersection(const YAML::Node& intersection_node,
                                                     const RoadRulebook& road_rulebook,
                                                     const PhaseRingBook& phase_ring_book,
                                                     ManualPhaseProvider* phase_provider) {
  MALIPUT_THROW_UNLESS(intersection_node.IsMap());
  MALIPUT_THROW_UNLESS(intersection_node["ID"].IsDefined());
  MALIPUT_THROW_UNLESS(intersection_node["PhaseRing"].IsDefined());
  MALIPUT_THROW_UNLESS(intersection_node["InitialPhase"].IsDefined());
  const Intersection::Id id(intersection_node["ID"].as<std::string>());
  const PhaseRing::Id ring_id(intersection_node["PhaseRing"].as<std::string>());
  const Phase::Id phase_id(intersection_node["InitialPhase"].as<std::string>());
  std::optional<PhaseRing> ring = phase_ring_book.GetPhaseRing(ring_id);
  MALIPUT_THROW_UNLESS(ring.has_value());
  MALIPUT_THROW_UNLESS(ring->phases().size() > 0);
  MALIPUT_THROW_UNLESS(ring->phases().find(phase_id) != ring->phases().end());
  std::optional<api::rules::Phase::Id> next_phase_id = std::nullopt;
  std::optional<double> duration_until = std::nullopt;
  std::vector<PhaseRing::NextPhase> next_phases = ring->next_phases().at(phase_id);
  if (next_phases.size() > 0) {
    // This arbitrarily selects the first (index 0) next phase. In the future,
    // we may want to more intelligently choose this.
    const PhaseRing::NextPhase n = next_phases.at(0);
    next_phase_id = n.id;
    duration_until = n.duration_until;
  }
  if (!phase_provider->GetPhase(ring_id).has_value()) {
    phase_provider->AddPhaseRing(ring_id, phase_id, next_phase_id, duration_until);
  } else {
    phase_provider->SetPhase(ring_id, phase_id, next_phase_id, duration_until);
  }
  // The following arbitrarily uses the first phase within the PhaseRing. This
  // is acceptable since a PhaseRing guarantees that all Phases within it share
  // the same domain.
  // TODO(#108): Remove GetRegionOfOldRules usage once old rules are deprecated.
  std::vector<LaneSRange> region = GetRegion(road_rulebook, ring->phases().begin()->second);
  region = region.empty() ? GetRegionOfOldRules(road_rulebook, ring->phases().begin()->second) : region;
  return std::make_unique<Intersection>(id, region, ring.value(), phase_provider);
}

std::unique_ptr<api::IntersectionBook> BuildFrom(const YAML::Node& root_node, const RoadRulebook& road_rulebook,
                                                 const PhaseRingBook& phase_ring_book,
                                                 const api::RoadGeometry* road_geometry,
                                                 ManualPhaseProvider* phase_provider) {
  MALIPUT_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& intersections_node = root_node["Intersections"];
  auto result = std::make_unique<IntersectionBook>(road_geometry);
  if (!intersections_node.IsDefined()) {
    return result;
  }
  MALIPUT_THROW_UNLESS(intersections_node.IsSequence());
  for (const YAML::Node& intersection_node : intersections_node) {
    result->AddIntersection(BuildIntersection(intersection_node, road_rulebook, phase_ring_book, phase_provider));
  }
  return result;
}

}  // namespace

std::unique_ptr<api::IntersectionBook> LoadIntersectionBook(const std::string& input, const RoadRulebook& road_rulebook,
                                                            const PhaseRingBook& phase_ring_book,
                                                            const api::RoadGeometry* road_geometry,
                                                            ManualPhaseProvider* phase_provider) {
  return BuildFrom(YAML::Load(input), road_rulebook, phase_ring_book, road_geometry, phase_provider);
}

std::unique_ptr<api::IntersectionBook> LoadIntersectionBookFromFile(const std::string& filename,
                                                                    const RoadRulebook& road_rulebook,
                                                                    const PhaseRingBook& phase_ring_book,
                                                                    const api::RoadGeometry* road_geometry,
                                                                    ManualPhaseProvider* phase_provider) {
  return BuildFrom(YAML::LoadFile(filename), road_rulebook, phase_ring_book, road_geometry, phase_provider);
}

}  // namespace maliput
