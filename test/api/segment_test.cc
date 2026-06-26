// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include "maliput/geometry_base/segment.h"

#include <memory>
#include <optional>

#include <gtest/gtest.h>

#include "maliput/geometry_base/junction.h"

namespace maliput {
namespace api {
namespace test {
namespace {

GTEST_TEST(SegmentTest, IsIntersectionDefaultsToUnavailableBeforeJunctionAttachment) {
  const geometry_base::Segment dut(SegmentId("segment"));
  EXPECT_EQ(std::nullopt, dut.is_intersection());
}

GTEST_TEST(SegmentTest, IsIntersectionFollowsJunctionClassification) {
  auto intersection_junction = std::make_unique<geometry_base::Junction>(JunctionId("intersection_junction"), true);
  auto intersection_segment = std::make_unique<geometry_base::Segment>(SegmentId("intersection_segment"));
  const geometry_base::Segment* intersection_segment_ptr =
      intersection_junction->AddSegment(std::move(intersection_segment));
  EXPECT_EQ(std::make_optional(true), intersection_segment_ptr->is_intersection());

  auto non_intersection_junction =
      std::make_unique<geometry_base::Junction>(JunctionId("non_intersection_junction"), false);
  auto non_intersection_segment = std::make_unique<geometry_base::Segment>(SegmentId("non_intersection_segment"));
  const geometry_base::Segment* non_intersection_segment_ptr =
      non_intersection_junction->AddSegment(std::move(non_intersection_segment));
  EXPECT_EQ(std::make_optional(false), non_intersection_segment_ptr->is_intersection());
}

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
