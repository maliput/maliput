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
#include "maliput/routing/compare.h"

#include <optional>
#include <string>

#include "maliput/api/compare.h"

namespace maliput {
namespace routing {

common::ComparisonResult<PhasePositionResult> IsPhasePositionResultClose(const PhasePositionResult& ppr_a,
                                                                         const PhasePositionResult& ppr_b,
                                                                         double tolerance) {
  if (ppr_a.lane_s_range_index != ppr_b.lane_s_range_index) {
    return {"PhasePositionResults are different at lane_s_range_index. ppr_a.lane_s_range_index: " +
            std::to_string(ppr_a.lane_s_range_index) +
            " vs. ppr_b.lane_s_range_index: " + std::to_string(ppr_b.lane_s_range_index) + "\n"};
  }
  const common::ComparisonResult<api::LanePosition> lane_position_compare_result =
      IsLanePositionClose(ppr_a.lane_position, ppr_b.lane_position, tolerance);
  if (lane_position_compare_result.message.has_value()) {
    return {lane_position_compare_result.message};
  }
  const common::ComparisonResult<api::InertialPosition> inertial_position_compare_result =
      IsInertialPositionClose(ppr_a.inertial_position, ppr_b.inertial_position, tolerance);
  if (inertial_position_compare_result.message.has_value()) {
    return {inertial_position_compare_result.message};
  }
  const double delta = std::abs(ppr_a.distance - ppr_b.distance);
  if (delta > tolerance) {
    return {"PhasePositionResults are different at distance. ppr_a.distance: " + std::to_string(ppr_a.distance) +
            " vs. ppr_b.distance: " + std::to_string(ppr_b.distance) + ", delta: " + std::to_string(delta) +
            ", tolerance: " + std::to_string(tolerance) + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<RoutePositionResult> IsRoutePositionResultClose(const RoutePositionResult& rpr_a,
                                                                         const RoutePositionResult& rpr_b,
                                                                         double tolerance) {
  if (rpr_a.phase_index != rpr_b.phase_index) {
    return {"RoutePositionResult are different at phase_index. rpr_a.phase_index: " +
            std::to_string(rpr_a.phase_index) + " vs. rpr_b.phase_index: " + std::to_string(rpr_b.phase_index) + "\n"};
  }
  return {IsPhasePositionResultClose(rpr_a.phase_position_result, rpr_b.phase_position_result, tolerance).message};
}

}  // namespace routing
}  // namespace maliput
