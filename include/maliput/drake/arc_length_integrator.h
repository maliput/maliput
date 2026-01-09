// BSD 3-Clause License
//
// Copyright (c) 2023-2026, Woven by Toyota.
// All rights reserved.
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

#include <functional>
#include <memory>

#include "maliput/common/maliput_copyable.h"
#include "maliput/drake/integrator_configuration.h"

namespace maliput {
namespace drake {

/// Wraps maliput::drake::systems::AntiderivativeFunction<double> by using a pImpl.
///
/// This convenient class helps to compute the arc length integral of the world
/// function given a parameter and returns either the corresponding image arc length
/// or a function that helps with multiple queries and return the arc length.
/// This class does not expose any drake / eigen specific type, which serves as
/// a useful interface for the future.
/// Backends can rely on linking against this library only and private linkage
/// will solve any required dependency.
class ArcLengthIntegrator final {
 public:
  /// Type alias for the a scalar integrable function f(x; 𝐤) type.
  ///
  /// @param x The variable of integration x ∈ ℝ .
  /// @param k The parameter vector 𝐤 ∈ ℝ².
  /// @return The function value f(@p x; @p k).
  using IntegrableFunction = std::function<double(const double& p, const math::Vector2& k)>;

  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ArcLengthIntegrator);

  /// Constructs an ArcLengthIntegrator.
  ///
  /// @param function A functor with the IntegrableFunction.
  /// @param config A IntegratorConfiguration with the initial state of the integrator
  /// and its configurations.
  ArcLengthIntegrator(const IntegrableFunction& function, const IntegratorConfiguration& config);
  ~ArcLengthIntegrator();

  /// Evaluates the arc length at @p p and with context @p k.
  ///
  /// @param p The parameter of the integrable function.
  /// @param k The context of the function.
  /// @return The arc length of the integrable function.
  double Evaluate(double p, const math::Vector2& k) const;

  /// Computes a functor that can be used for sucessive queries at different
  /// p parameter values of the arc length.
  ///
  /// @param p0 The start of the integration domain.
  /// @param p1 The end of the integration domain.
  /// @param p1 The context k of the integration domain.
  /// @param tolerance The absolute tolerance to determine whether the used
  /// p parameter is within range [@p p0, @p p1].
  /// @return A std::function that converts p values into the corresponding arc length.
  std::function<double(double)> IntegralFunction(double p0, double p1, const math::Vector2& k, double tolerance) const;

 private:
  /// Holds the internal data model to satisfy the pImpl pattern.
  struct ArcLengthIntegratorDataImpl;

  std::unique_ptr<ArcLengthIntegratorDataImpl> impl_;
};

}  // namespace drake
}  // namespace maliput
