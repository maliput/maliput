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

#include <optional>
#include <unordered_map>
#include <vector>

#include "maliput/api/rules/phase.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// A set of mutually exclusive phases, e.g., that comprise the signaling
/// cycle for an intersection.
class PhaseRing final {
 public:
  /// Holds a "next phase" specification. This is one of the phases that could
  /// be next after the current phase ends.
  struct NextPhase {
    /// The ID of the next phase.
    Phase::Id id;

    /// The default time before transitioning to the next phase. This is
    /// relative to when the current phase began. It is just a recommendation,
    /// the actual duration is determined by the PhaseProvider and may depend on
    /// events like a vehicle arriving at a left-turn lane or a pedestrian
    /// hitting a crosswalk button.
    std::optional<double> duration_until;
  };

  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PhaseRing);

  /// Unique identifier for a PhaseRing.
  using Id = TypeSpecificIdentifier<class PhaseRing>;

  /// Constructs a PhaseRing.
  ///
  /// @param id the unique ID of this phase ring
  /// @param phases the phases within this ring.
  /// @param next_phases specifies, for each phase, possible next phases. This
  /// can be std::nullopt, in which case, no next phases will be specified.
  ///
  /// @throws std::exception if `phases` is empty, `phases` contains duplicate
  /// Phase::Id's, the phases define different sets of RightOfWayRule::Ids and
  /// Rule::Ids, or the phases define different sets of bulb states. If
  /// `next_phases` is provided, this method will also throw if `next_phases`
  /// does not define the possible next phases of every phase in `phases`, or
  /// defines a next phase that is not in `phases`.
  PhaseRing(
      const Id& id, const std::vector<Phase>& phases,
      const std::optional<const std::unordered_map<Phase::Id, std::vector<NextPhase>>>& next_phases = std::nullopt);

  /// Returns the phase ring's identifier.
  const Id& id() const { return id_; }

  /// Returns a Phase by its `id` or std::nullopt when it is not present in this
  /// PhaseRing.
  std::optional<Phase> GetPhase(const Phase::Id& id) const;

  /// Returns the catalog of phases.
  const std::unordered_map<Phase::Id, Phase>& phases() const { return phases_; }

  const std::unordered_map<Phase::Id, std::vector<NextPhase>>& next_phases() const { return next_phases_; }

  /// Returns the possible next phases given the current phase with an ID of
  /// `id`.
  ///
  /// @throws std::out_of_range if no phase with an ID of `id` exists.
  const std::vector<NextPhase>& GetNextPhases(const Phase::Id& id) const { return next_phases_.at(id); }

 private:
  Id id_;
  std::unordered_map<Phase::Id, Phase> phases_;
  std::unordered_map<Phase::Id, std::vector<NextPhase>> next_phases_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
