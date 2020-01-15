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
