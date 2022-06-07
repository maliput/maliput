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

#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {

/// A simple concrete implementation of the api::rules::PhaseRingBook abstract
/// interface. It allows users to obtain the ID of the PhaseRing that includes a
/// particular RightOfWayRule.
class SimplePhaseRingBook : public api::rules::PhaseRingBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(SimplePhaseRingBook);

  SimplePhaseRingBook();

  ~SimplePhaseRingBook() override;

  /// Adds @p ring to this SimplePhaseRingBook.
  ///
  /// @throws std::exception if an api::rules::PhaseRing with the same ID
  /// already exists, or if @p ring contains a rule that already exists in a
  /// previously added api::rules::PhaseRing.
  void AddPhaseRing(const api::rules::PhaseRing& ring);

  /// Removes an api::rules::PhaseRing with an ID of @p ring_id from this
  /// SimplePhaseRingBook.
  ///
  /// @throws std::exception if the specified api::rules::PhaseRing does not
  /// exist.
  void RemovePhaseRing(const api::rules::PhaseRing::Id& ring_id);

 private:
  std::optional<api::rules::PhaseRing> DoGetPhaseRing(const api::rules::PhaseRing::Id& ring_id) const override;

  std::optional<api::rules::PhaseRing> DoFindPhaseRing(const api::rules::RightOfWayRule::Id& rule_id) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
