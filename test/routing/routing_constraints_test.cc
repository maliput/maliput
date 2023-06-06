// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
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
#include "maliput/routing/routing_constraints.h"

#include <optional>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

namespace maliput {
namespace routing {
namespace test {
namespace {

TEST(ValidateRoutingConstraintsTest, AllElementsAreValidDoesNotThrow) {
  constexpr RoutingConstraints kDutA{
      true, /* allow_lane_switch */
      {}    /* max_routing_phase_cost */
  };
  constexpr RoutingConstraints kDutB{
      false, /* allow_lane_switch */
      {100.} /* max_routing_phase_cost */
  };

  EXPECT_NO_THROW(ValidateRoutingConstraints(kDutA));
  EXPECT_NO_THROW(ValidateRoutingConstraints(kDutB));
}

TEST(ValidateRoutingConstraintsTest, MaxRoutingPhaseCostNegativeMakesItThrow) {
  constexpr RoutingConstraints kDut{
      false,  /* allow_lane_switch */
      {-100.} /* max_routing_phase_cost */
  };

  EXPECT_THROW({ ValidateRoutingConstraints(kDut); }, common::assertion_error);
}

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
