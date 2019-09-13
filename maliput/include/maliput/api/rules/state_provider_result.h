#pragma once

#include "drake/common/drake_optional.h"

namespace maliput {
namespace api {
namespace rules {

/// The state returned by a state provider.
///
/// Each provider must define the final type @tparam T.
template <class T>
struct StateProviderResult {
  /// Information about the next state.
  struct Next {
    T state;
    /// If known, the estimated time until the transition to the next state,
    /// relative to when the state is queried to a state provider.
    /// Users should treat this as advisory since it is tentative and
    /// subject to change at any point in time.
    drake::optional<double> duration_until;
  };

  /// The rule's current state.
  T state;
  /// The rule's next state, if known. Users should treat this as advisory
  /// since it is tentative and subject to change at any point in time.
  drake::optional<Next> next;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
