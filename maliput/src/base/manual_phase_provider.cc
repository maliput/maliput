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
#include "maliput/base/manual_phase_provider.h"

#include <stdexcept>
#include <string>
#include <unordered_map>

#include "maliput/common/maliput_throw.h"

namespace maliput {

using api::rules::Phase;
using api::rules::PhaseProvider;
using api::rules::PhaseRing;

class ManualPhaseProvider::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() {}
  ~Impl() {}

  void AddPhaseRing(const PhaseRing::Id& id, const Phase::Id& initial_phase,
                    const std::optional<Phase::Id>& initial_next_phase,
                    const std::optional<double>& initial_duration_until) {
    if (initial_duration_until.has_value() && !initial_next_phase.has_value()) {
      throw std::logic_error(
          "Initial duration-until specified when initial next phase is "
          "unspecified.");
    }
    auto result = phases_.emplace(
        id, PhaseProvider::Result{initial_phase, ComputeNext(initial_next_phase, initial_duration_until)});
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple phase rings with id " + id.string());
    }
  }

  void SetPhase(const PhaseRing::Id& id, const Phase::Id& phase, const std::optional<Phase::Id>& next_phase,
                const std::optional<double>& duration_until) {
    if (duration_until.has_value() && !next_phase.has_value()) {
      throw std::logic_error("Duration-until specified when next phase is unspecified.");
    }
    MALIPUT_THROW_UNLESS(phases_.find(id) != phases_.end());
    phases_.at(id) = PhaseProvider::Result{phase, ComputeNext(next_phase, duration_until)};
  }

  std::optional<PhaseProvider::Result> DoGetPhase(const PhaseRing::Id& id) const {
    auto it = phases_.find(id);
    if (it == phases_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

 private:
  std::optional<PhaseProvider::Result::Next> ComputeNext(const std::optional<Phase::Id>& next_phase,
                                                         const std::optional<double>& duration_until) const {
    std::optional<PhaseProvider::Result::Next> result = std::nullopt;
    if (next_phase.has_value()) {
      result = PhaseProvider::Result::Next{*next_phase, duration_until};
    }
    return result;
  }

  std::unordered_map<maliput::PhaseRing::Id, PhaseProvider::Result> phases_;
};

ManualPhaseProvider::ManualPhaseProvider() : impl_(std::make_unique<Impl>()) {}

ManualPhaseProvider::~ManualPhaseProvider() = default;

void ManualPhaseProvider::AddPhaseRing(const PhaseRing::Id& id, const Phase::Id& initial_phase,
                                       const std::optional<Phase::Id>& initial_next_phase,
                                       const std::optional<double>& initial_duration_until) {
  impl_->AddPhaseRing(id, initial_phase, initial_next_phase, initial_duration_until);
}

void ManualPhaseProvider::SetPhase(const PhaseRing::Id& id, const Phase::Id& phase,
                                   const std::optional<Phase::Id>& next_phase,
                                   const std::optional<double>& duration_until) {
  impl_->SetPhase(id, phase, next_phase, duration_until);
}

std::optional<PhaseProvider::Result> ManualPhaseProvider::DoGetPhase(const api::rules::PhaseRing::Id& id) const {
  return impl_->DoGetPhase(id);
}

}  // namespace maliput
