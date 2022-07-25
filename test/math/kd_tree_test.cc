// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
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
#include "maliput/math/kd_tree.h"

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace math {
namespace {

class KDTreeTest : public ::testing::Test {
 public:
};

TEST_F(KDTreeTest, Constructor) {
  const std::vector<maliput::math::Vector3> points{};
  ASSERT_THROW((KDTree<maliput::math::Vector3, 3>(points.begin(), points.end())), maliput::common::assertion_error);
  ASSERT_NO_THROW((KDTree<maliput::math::Vector3, 3>(std::vector<maliput::math::Vector3>{{3, 6, 2}})));
}

TEST_F(KDTreeTest, NNSearch) {
  std::vector<maliput::math::Vector3> points{{6, 5, 2}, {1, 8, 9}, {7, 1, 1}, {7, 8, 7}, {7, 3, 6},
                                             {2, 1, 4}, {1, 4, 1}, {8, 7, 3}, {6, 2, 8}, {2, 8, 6},
                                             {9, 1, 4}, {2, 8, 3}, {8, 1, 4}, {9, 4, 1}, {7, 2, 9}};
  const KDTree<maliput::math::Vector3, 3> dut{points.begin(), points.end()};
  const maliput::math::Vector3 point{3., 3., 3.};
  const maliput::math::Vector3 expected_point{2., 1., 4.};
  const double expected_distance{(point - expected_point).norm()};

  // TODO(): Modify API to return a NNSearchResult struct with the point, distance, and visited nodes.
  const auto nearest_point = dut.Nearest(point);
  // Tests point being returned.
  EXPECT_DOUBLE_EQ(expected_point.x(), nearest_point.x());
  EXPECT_DOUBLE_EQ(expected_point.y(), nearest_point.y());
  EXPECT_DOUBLE_EQ(expected_point.z(), nearest_point.z());
  // Tests distance being returned.
  EXPECT_DOUBLE_EQ(expected_distance, dut.Distance());
  // Tests that visited nodes are less than brute force.
  EXPECT_LT(dut.Visited(), points.size());
}

}  // namespace
}  // namespace math
}  // namespace maliput
