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

#include <unordered_map>
#include <vector>

#include "maliput/api/regions.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

// TODO(andrew.best@tri.global): Add support for multiple states. Currently,
// it's enforced that all rules have exactly one state and are therefore
// static. To support multiple states, a StateProvider is needed.
/// Rule describing direction usage for a road lane.
///
/// DirectionUsageRules are comprised of:
/// * a zone (a LaneSRange) which specifies the longitudinal section of the
/// road-network to which the rule instance applies.
/// * a catalog of one or more States, each of which indicate the possible
///   DirectionUsageRule semantics for a vehicle traversing the zone.
///
/// A rule instance with a single State is considered "static", and has fixed
/// semantics.
///
/// Each Lane location can be governed by at most one DirectionUsageRule.
class MALIPUT_DEPRECATED("DirectionUsageRule will be deprecated", "DiscreteValueRule") DirectionUsageRule final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirectionUsageRule);

  using Id = TypeSpecificIdentifier<class DirectionUsageRule>;

  /// Semantic state of the DirectionUsageRule.
  /// A State describes the current usage semantics of the lane section.
  /// This includes which direction traffic is allowed to travel on the lane
  /// and the severity of this restriction.
  class State final {
   public:
    MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    /// Unique identifier for a State
    using Id = TypeSpecificIdentifier<class State>;

    enum class Severity {
      /// No vehicle should travel on this lane in violation of this rule.
      kStrict = 0,
      /// Vehicles should avoid travelling against this rule but certain
      /// exceptions apply. E.g. passing is allowed.
      kPreferred,
    };

    /// Type of allowed travel on the lane. Categorized based on travel with or
    /// against the central axis (+S) of the lane frame.
    enum class Type {
      /// Travel should proceed in the direction of the +S axis.
      kWithS = 0,
      /// Travel should proceed opposite the +S axis direction.
      kAgainstS,
      /// Travel is allowed both with the lane direction(+S) or against it.
      kBidirectional,
      /// Travel is allowed both with the lane direction(+S) or against it but
      /// should be limited in duration, e.g. when approaching turns.
      kBidirectionalTurnOnly,
      /// Travel on this lane is prohibited.
      kNoUse,
      /// This lane is used to define a parking area.
      kParking,
      /// There is no defined direction of travel on this lane.
      kUndefined,
    };

    /// Constructs a State instance.
    ///
    /// @param id the unique Id
    /// @param type the semantic Type
    /// @param severity the Severity of the State
    State(Id id, Type type, Severity severity) : id_(id), type_(type), severity_(severity) {}

    /// Returns the Id.
    const Id& id() const { return id_; }

    /// Returns the Type.
    Type type() const { return type_; }

    /// Returns the Severity.
    Severity severity() const { return severity_; }

   private:
    Id id_;
    Type type_{};
    Severity severity_{};
  };

  // TODO(andrew.best@tri.global): Enable StateProvider and remove the one state
  //                               restriction.
  /// Constructs a DirectionUsageRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param states a vector of valid states for the rule
  /// @throws maliput::common::assertion_error if size of states is not exactly
  ///         1.
  DirectionUsageRule(const Id& id, const LaneSRange& zone, std::vector<State> states) : id_(id), zone_(zone) {
    MALIPUT_VALIDATE(states.size() >= 1, "DirectionUsageRule(" + id_.string() + ") must have at least one state.");
    for (const State& state : states) {
      // Construct index of states by ID, ensuring uniqueness of ID's.
      auto result = states_.emplace(state.id(), state);
      MALIPUT_THROW_UNLESS(result.second);
    }
  }

  /// Returns the persistent identifier.
  const Id& id() const { return id_; }

  /// Returns the zone to which this rule instance applies.
  const LaneSRange& zone() const { return zone_; }

  /// Returns the catalog of possible States.
  const std::unordered_map<State::Id, State>& states() const { return states_; }

  /// Returns true if the rule is static, i.e. has only one state,
  /// otherwise false.
  bool is_static() const { return states_.size() == 1; }

  /// Returns the static state of the rule.
  ///
  /// This is a convenience function for returning a static rule's single state.
  ///
  /// @throws maliput::common::assertion_error if `is_static()` is false.
  const State& static_state() const {
    MALIPUT_VALIDATE(is_static(),
                     "Calling DirectionUsageRule(" + id_.string() + ")::static_state() but the state is not static.");
    return states_.begin()->second;
  }

  /// Maps DirectionUsageRule::State::Type enums to string representations.
  static std::unordered_map<State::Type, const char*, maliput::common::DefaultHash> StateTypeMapper();

 private:
  Id id_;
  LaneSRange zone_;
  std::unordered_map<State::Id, State> states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
