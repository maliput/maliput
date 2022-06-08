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
#include <memory>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "maliput/api/segment.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/utility/segment_analysis.h"

namespace maliput {
namespace utility {
namespace {

GTEST_TEST(MockSegmentAnalysisAnalyzeConfluentSegments, OneLaneRoadGeometry) {
  const std::unique_ptr<const api::RoadGeometry> rg = api::test::CreateOneLaneRoadGeometry();

  const std::vector<std::unordered_set<const api::Segment*>> groups = AnalyzeConfluentSegments(rg.get());

  EXPECT_TRUE(groups.empty());
}

GTEST_TEST(MockSegmentAnalysisAnalyzeConfluentSegments, TwoLanesRoadGeometry) {
  const std::unique_ptr<const api::RoadGeometry> rg = api::test::CreateTwoLanesRoadGeometry();

  const std::vector<std::unordered_set<const api::Segment*>> groups = AnalyzeConfluentSegments(rg.get());

  const std::set<std::set<const api::Segment*>> expected{{}, {}};

  // Recast groups as "set of sets" (instead of "vector of unordered_sets")
  // to make equality testing trivial.
  std::set<std::set<const api::Segment*>> actual;
  for (const auto& group : groups) {
    std::set<const api::Segment*> set(group.begin(), group.end());
    actual.insert(set);
  }
  EXPECT_EQ(actual, expected);
}

}  // anonymous namespace
}  // namespace utility
}  // namespace maliput
