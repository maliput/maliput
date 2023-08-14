// BSD 3-Clause License
//
// Copyright (c) 2023, Woven Planet. All rights reserved.
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

#include <gtest/gtest.h>

#include "maliput/routing/route_position_result.h"

namespace maliput {
namespace routing {
namespace test {

/// Compares equality within @p tolerance deviation of the PhasePositionResult @p ppr_a and @p ppr_b.
/// @param ppr_a The first PhasePositionResult to compare.
/// @param ppr_b The second PhasePositionResult to compare.
/// @param tolerance The tolerance to use for the comparison.
/// @return An ::testing::AssertionResult with the result of the comparison.
::testing::AssertionResult IsPhasePositionResultClose(const PhasePositionResult& ppr_a,
                                                      const PhasePositionResult& ppr_b, double tolerance);

/// Compares equality within @p tolerance deviation of the RoutePositionResult @p ppr_a and @p ppr_b.
/// @param rpr_a The first RoutePositionResult to compare.
/// @param rpr_b The second RoutePositionResult to compare.
/// @param tolerance The tolerance to use for the comparison.
/// @return An ::testing::AssertionResult with the result of the comparison.
::testing::AssertionResult IsRoutePositionResultClose(const RoutePositionResult& rpr_a,
                                                      const RoutePositionResult& rpr_b, double tolerance);

}  // namespace test
}  // namespace routing
}  // namespace maliput
