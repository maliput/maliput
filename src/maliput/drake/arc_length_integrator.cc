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
#include "maliput/drake/arc_length_integrator.h"

#include <Eigen/Dense>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/drake/common/eigen_types.h>
#include <maliput/drake/systems/analysis/antiderivative_function.h>
#include <maliput/drake/systems/analysis/integrator_base.h>
#include <maliput/math/saturate.h>
#include <maliput/math/vector.h>

namespace maliput {
namespace drake {
namespace {

struct AntiDerivativeFunctionWrapper {
  explicit AntiDerivativeFunctionWrapper(const ArcLengthIntegrator::IntegrableFunction& f_in) : f(f_in) {}

  double operator()(double p, const VectorX<double>& k) const {
    const math::Vector2 math_k(k(0), k(1));
    return f(p, math_k);
  }

  std::function<double(double, const math::Vector2&)> f;
};

}  // namespace

struct ArcLengthIntegrator::ArcLengthIntegratorDataImpl {
  std::unique_ptr<systems::AntiderivativeFunction<double>> antiderivative_function{};
};

ArcLengthIntegrator::ArcLengthIntegrator(const ArcLengthIntegrator::IntegrableFunction& function,
                                         const IntegratorConfiguration& config) {
  impl_ = std::make_unique<ArcLengthIntegratorDataImpl>();

  // Construct the AntiderivativeFunction
  const VectorX<double> default_parameters = (VectorX<double>(2) << config.k[0], config.k[1]).finished();
  const systems::AntiderivativeFunction<double>::IntegrableFunctionContext antiderivative_function_values(
      config.parameter_lower_bound, default_parameters);
  impl_->antiderivative_function = std::make_unique<systems::AntiderivativeFunction<double>>(
      AntiDerivativeFunctionWrapper(function), antiderivative_function_values);
  // Configure the integrator.
  systems::IntegratorBase<double>& integrator = impl_->antiderivative_function->get_mutable_integrator();
  integrator.request_initial_step_size_target(config.initial_step_size_target);
  integrator.set_maximum_step_size(config.maximum_step_size);
  integrator.set_target_accuracy(config.target_accuracy);
}

ArcLengthIntegrator::~ArcLengthIntegrator() = default;

double ArcLengthIntegrator::Evaluate(double p, const math::Vector2& k) const {
  // Populates parameter vector with (r, h) coordinate values.
  systems::AntiderivativeFunction<double>::IntegrableFunctionContext context;
  context.k = (VectorX<double>(2) << k[0], k[1]).finished();
  return impl_->antiderivative_function->Evaluate(p, context);
}

std::function<double(double)> ArcLengthIntegrator::IntegralFunction(double p0, double p1, const math::Vector2& k,
                                                                    double tolerance) const {
  // Populates parameter vector with (r, h) coordinate values.
  systems::AntiderivativeFunction<double>::IntegrableFunctionContext context;
  context.k = (VectorX<double>(2) << k[0], k[1]).finished();
  // Prepares dense output for shared ownership, as std::function
  // instances only take copyable callables.
  const std::shared_ptr<systems::ScalarDenseOutput<double>> dense_output{
      impl_->antiderivative_function->MakeDenseEvalFunction(p1, context)};
  MALIPUT_THROW_UNLESS(dense_output->start_time() <= p0);
  MALIPUT_THROW_UNLESS(dense_output->end_time() >= p1);
  return [dense_output, tolerance, p0, p1](double p) -> double {
    // Saturates p to lie within the [p0, p1] interval.
    const double saturated_p = maliput::math::saturate(p, p0, p1);
    MALIPUT_THROW_UNLESS(std::abs(saturated_p - p) < tolerance);
    return dense_output->EvaluateScalar(saturated_p);
  };
}

}  // namespace drake
}  // namespace maliput
