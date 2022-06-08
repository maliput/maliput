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
#include "maliput/test_utilities/check_id_indexing.h"

#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {

namespace rules {
namespace test {
template <typename T>
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const T* a, const T* b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}
}  // namespace test
}  // namespace rules

namespace test {

::testing::AssertionResult CheckIdIndexing(const RoadGeometry* road_geometry) {
  rules::test::AssertionResultCollector c;
  for (int ji = 0; ji < road_geometry->num_junctions(); ++ji) {
    const api::Junction* junction = road_geometry->junction(ji);
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(road_geometry->ById().GetJunction(junction->id()), junction));
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(road_geometry->ById().GetSegment(segment->id()), segment));
      for (int li = 0; li < segment->num_lanes(); ++li) {
        const api::Lane* lane = segment->lane(li);
        MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(road_geometry->ById().GetLane(lane->id()), lane));
      }
    }
    for (int bi = 0; bi < road_geometry->num_branch_points(); ++bi) {
      const api::BranchPoint* branch_point = road_geometry->branch_point(bi);
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(road_geometry->ById().GetBranchPoint(branch_point->id()), branch_point));
    }
  }
  return c.result();
}

}  // namespace test
}  // namespace api
}  // namespace maliput
