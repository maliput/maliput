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
#include <vector>

#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the mapping from RightOfWayRule::Id to
/// PhaseRing.
class PhaseRingBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseRingBook);

  virtual ~PhaseRingBook() = default;

  /// Gets a list of all PhaseRings within this book.
  std::vector<PhaseRing::Id> GetPhaseRings() const { return DoGetPhaseRings(); }

  /// Gets the specified PhaseRing. Returns std::nullopt if @p ring_id is
  /// unrecognized.
  std::optional<PhaseRing> GetPhaseRing(const PhaseRing::Id& ring_id) const { return DoGetPhaseRing(ring_id); }

  /// Finds and returns the PhaseRing containing the specified
  /// RightOfWayRule. Returns std::nullopt if @p rule_id is unrecognized.
  MALIPUT_DEPRECATED("RightOfWayRule class will be deprecated.")
  std::optional<PhaseRing> FindPhaseRing(const RightOfWayRule::Id& rule_id) const { return DoFindPhaseRing(rule_id); }

  /// Finds and returns the PhaseRing containing the specified
  /// Rule. Returns std::nullopt if @p rule_id is unrecognized.
  std::optional<PhaseRing> FindPhaseRing(const Rule::Id& rule_id) const { return DoFindPhaseRing(rule_id); }

 protected:
  PhaseRingBook() = default;

 private:
  virtual std::vector<PhaseRing::Id> DoGetPhaseRings() const = 0;

  virtual std::optional<PhaseRing> DoGetPhaseRing(const PhaseRing::Id& ring_id) const = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  virtual std::optional<PhaseRing> DoFindPhaseRing(const RightOfWayRule::Id& rule_id) const = 0;
#pragma GCC diagnostic pop

  virtual std::optional<PhaseRing> DoFindPhaseRing(const Rule::Id& rule_id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
