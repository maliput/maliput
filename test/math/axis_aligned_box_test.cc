// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
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
#include "maliput/math/axis_aligned_box.h"

#include <cmath>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput/math/overlapping_type.h"

namespace maliput {
namespace math {
namespace test {
namespace {

static constexpr double kTolerance{1e-12};

TEST(AxisAlignedBox, Constructors) {
  // Throws due to a negative tolerance value.
  EXPECT_THROW(AxisAlignedBox({std::numeric_limits<double>::infinity(), 2., 3.} /* min_corner */,
                              {4., 5., 6.} /* max_corner */, -1 /* tolerance */),
               maliput::common::assertion_error);
  // Throws due to a invalid min_corner/max_corner values
  EXPECT_THROW(AxisAlignedBox({5. /* greater than max_corner.x() */, 2., 3.} /* min_corner */,
                              {4., 5., 6.} /* max_corner */, 1e-12 /* tolerance */),
               maliput::common::assertion_error);
  // Throws due to a invalid min_corner/max_corner values
  EXPECT_THROW(AxisAlignedBox({1., 6. /* greater than max_corner.x() */, 3.} /* min_corner */,
                              {4., 5., 6.} /* max_corner */, 1e-12 /* tolerance */),
               maliput::common::assertion_error);
  // Throws due to a invalid min_corner/max_corner values
  EXPECT_THROW(AxisAlignedBox({1., 2., 7. /* greater than max_corner.x() */} /* min_corner */,
                              {4., 5., 6.} /* max_corner */, 1e-12 /* tolerance */),
               maliput::common::assertion_error);
  // It is all correct.
  EXPECT_NO_THROW(AxisAlignedBox({1., 2., 3.} /* min_corner */, {4., 5., 6.} /* max_corner */, 1e-12 /* tolerance */));
  // It supports infinite values.
  EXPECT_NO_THROW(AxisAlignedBox({-std::numeric_limits<double>::infinity(), 2., 3.} /* min_corner */,
                                 {4., 5., 6.} /* max_corner */, 1e-12 /* tolerance */));
  EXPECT_NO_THROW(AxisAlignedBox({1, 2., 3.} /* min_corner */,
                                 {4., 5., std::numeric_limits<double>::infinity()} /* max_corner */,
                                 1e-12 /* tolerance */));
}

struct ExpectedPositionContainsResults {
  Vector3 position{};
  bool contains_position{};
};

struct ExpectedGetIntersectionResults {
  AxisAlignedBox other_box;
  std::optional<AxisAlignedBox> resulted_intersection;
};

struct AxisAlignedBoxCase {
  Vector3 position{};
  Vector3 min_corner{};
  Vector3 max_corner{};

  std::vector<ExpectedPositionContainsResults> expected_position_contains_results{};
  std::vector<ExpectedGetIntersectionResults> expected_get_intersection_results{};
};

std::vector<AxisAlignedBoxCase> GetTestParameters() {
  return {
      // Centered at origin
      AxisAlignedBoxCase{{0., 0., 0.} /* position */,
                         {-1., -1., -1.} /* min_corner */,
                         {1., 1., 1.} /* max_corner */,
                         {
                             {{0., 0., 0}, true},   /* Right in the center */
                             {{1., 1., 1}, true},   /* Right in a vertex */
                             {{-1., 1., 1}, true},  /* Right in a vertex */
                             {{1., -1., 1}, true},  /* Right in a vertex */
                             {{1., 1., -1}, true},  /* Right in a vertex */
                             {{2., 1., 1.}, false}, /*A bit off on x*/
                             {{1., 2., 1.}, false}, /*A bit off on y*/
                             {{1., 1., 2.}, false}, /*A bit off on z*/
                         },
                         {
                             // No intersection.
                             {
                                 AxisAlignedBox{{1.1, 1.1, 1.1} /* min_corner */, {2., 2., 2.} /* max_corner */},
                                 std::nullopt,
                             },
                             // Intersection with a box that is contained in the other box.
                             {
                                 AxisAlignedBox{{-0.5, -0.5, -0.5} /* min_corner */, {0.5, 0.5, 0.5} /* max_corner */},
                                 AxisAlignedBox{{-0.5, -0.5, -0.5} /* min_corner */, {0.5, 0.5, 0.5} /* max_corner */},
                             },
                             // Intersection with a box that contains the other box.
                             {
                                 AxisAlignedBox{{-5., -5., -5.} /* min_corner */, {5., 5., 5.} /* max_corner */},
                                 AxisAlignedBox{{-1., -1., -1.} /* min_corner */, {1., 1., 1.} /* max_corner */},
                             },
                             // Intersection with a box that is partially contained in the other box.
                             {
                                 AxisAlignedBox{{0.5, 0.5, 0.5} /* min_corner */, {1.5, 1.5, 1.5} /* max_corner */},
                                 AxisAlignedBox{{0.5, 0.5, 0.5} /* min_corner */, {1., 1., 1.} /* max_corner */},
                             },
                             // Intersection with a box that is partially contained in the other box.
                             {
                                 AxisAlignedBox{{-1.5, -1.5, -1.5} /* min_corner */, {0., 0., 0.} /* max_corner */},
                                 AxisAlignedBox{{-1., -1., -1.} /* min_corner */, {0., 0., 0.} /* max_corner */},
                             },
                         }},
      // Centered away from origin
      AxisAlignedBoxCase{{10., 10., 10.} /* position */,
                         {9., 9., 9.} /* min_corner */,
                         {11., 11., 11.} /* max_corner */,
                         {
                             {{10., 10., 10}, true},   /* Right in the center */
                             {{11., 11., 11}, true},   /* Right in a vertex */
                             {{9., 11., 11}, true},    /* Right in a vertex */
                             {{11., 9., 11}, true},    /* Right in a vertex */
                             {{11., 11., 9}, true},    /* Right in a vertex */
                             {{12., 11., 11.}, false}, /*A bit of on x*/
                             {{11., 12., 11.}, false}, /*A bit of on y*/
                             {{11., 11., 12.}, false}, /*A bit of on z*/
                         },
                         {
                             // No GetIntersection tests for this case.
                         }},
      // The entire 3d plane.
      AxisAlignedBoxCase{{NAN, NAN, NAN} /* position */,
                         {std::numeric_limits<double>::infinity() * -1, std::numeric_limits<double>::infinity() * -1,
                          std::numeric_limits<double>::infinity() * -1} /* min_corner */,
                         {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity()} /* max_corner */,
                         {
                             {{10., 10., 10}, true},
                             {{-10., -10., -10}, true},
                             {{-100000., -100000., -100000}, true},
                             {{100000., 100000., 100000}, true},
                         },
                         {
                             // Any box that intersects the entire 3d plane should be the box.
                             {
                                 AxisAlignedBox{{-1., -1., -1.} /* min_corner */, {1., 1., 1.} /* max_corner */},
                                 AxisAlignedBox{{-1., -1., -1.} /* min_corner */, {1., 1., 1.} /* max_corner */},
                             },
                         }},
      // A quadrant: -x, -y, -z.
      AxisAlignedBoxCase{{-INFINITY, -INFINITY, -INFINITY} /* position */,
                         {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity()} /* min_corner */,
                         {0., 0., 0.} /* max_corner */,
                         {
                             {{-10., -10., -10.}, true},
                             {{-100000., -100000., -100000.}, true},
                             {{0., 0., 0.}, true},
                             {{10., 10., 10.}, false},
                             {{-10., 10., 10.}, false},
                             {{10., -10., 10.}, false},
                             {{10., 10., -10.}, false},
                             {{10., -10., -10.}, false},
                             {{-10., -10., 10.}, false},
                             {{100000., 100000., 100000}, false},
                         },
                         {
                             // No intersection.
                             {
                                 AxisAlignedBox{{1., 1., 1.} /* min_corner */, {2., 2., 2.} /* max_corner */},
                                 std::nullopt,
                             },
                             // A box that intersects partially the quadrant.
                             {
                                 AxisAlignedBox{{-1., -1., -1.} /* min_corner */, {1., 1., 1.} /* max_corner */},
                                 AxisAlignedBox{{-1., -1., -1.} /* min_corner */, {0., 0., 0.} /* max_corner */},
                             },
                         }},
      // -x -> +x
      AxisAlignedBoxCase{
          {-NAN, 5., 5.} /* position */,
          {-std::numeric_limits<double>::infinity(), 0, 0} /* min_corner */,
          {std::numeric_limits<double>::infinity(), 10., 10.} /* max_corner */,
          {
              {{5., 5., 5.}, true},
              {{500000., 5., 5.}, true},
              {{50., 15., 15.}, false},
          },
          {
              // No intersection.
              {
                  AxisAlignedBox{{-5., 20., 20.} /* min_corner */, {5., 30., 30.} /* max_corner */},
                  std::nullopt,
              },
              // Intersects
              {
                  AxisAlignedBox{{0., 0., 0.} /* min_corner */, {10., 10., 10.} /* max_corner */},
                  AxisAlignedBox{{0., 0., 0.} /* min_corner */, {10., 10., 10.} /* max_corner */},
              },
              // Intersects
              {
                  AxisAlignedBox{{-200., -200., -200.} /* min_corner */, {200., 200., 200.} /* max_corner */},
                  AxisAlignedBox{{-200., 0., 0.} /* min_corner */, {200., 10., 10.} /* max_corner */},
              },
          }},

  };
}

class AxisAlignedBoxTest : public ::testing::TestWithParam<AxisAlignedBoxCase> {
 public:
  void SetUp() override {}
  const AxisAlignedBoxCase case_ = GetParam();
};

TEST_P(AxisAlignedBoxTest, Getters) {
  const AxisAlignedBox dut{case_.min_corner, case_.max_corner, kTolerance};
  EXPECT_EQ(case_.min_corner, dut.min_corner());
  EXPECT_EQ(case_.max_corner, dut.max_corner());
}

TEST_P(AxisAlignedBoxTest, ContainsPosition) {
  const AxisAlignedBox dut{case_.min_corner, case_.max_corner, kTolerance};

  for (const auto expected_result : case_.expected_position_contains_results) {
    EXPECT_EQ(expected_result.contains_position, dut.Contains(expected_result.position))
        << "Expected contains value: " + std::string(expected_result.contains_position ? "True" : "False") +
               " at position: " + expected_result.position.to_str();
  }
}

TEST_P(AxisAlignedBoxTest, GetPosition) {
  const AxisAlignedBox dut{case_.min_corner, case_.max_corner, kTolerance};
  if (std::isnan(case_.position.x())) {
    EXPECT_TRUE(std::isnan(dut.position().x()));
  } else {
    EXPECT_EQ(case_.position.x(), dut.position().x());
  }
  if (std::isnan(case_.position.y())) {
    EXPECT_TRUE(std::isnan(dut.position().y()));
  } else {
    EXPECT_EQ(case_.position.y(), dut.position().y());
  }
  if (std::isnan(case_.position.z())) {
    EXPECT_TRUE(std::isnan(dut.position().z()));
  } else {
    EXPECT_EQ(case_.position.z(), dut.position().z());
  }
}

TEST_P(AxisAlignedBoxTest, GetIntersection) {
  const AxisAlignedBox dut{case_.min_corner, case_.max_corner, kTolerance};
  for (const auto expected_result : case_.expected_get_intersection_results) {
    const auto intersection_box = dut.GetIntersection(expected_result.other_box);
    EXPECT_EQ(expected_result.resulted_intersection.has_value(), intersection_box.has_value());
    if (intersection_box.has_value()) {
      EXPECT_EQ(expected_result.resulted_intersection->min_corner(), intersection_box->min_corner());
      EXPECT_EQ(expected_result.resulted_intersection->max_corner(), intersection_box->max_corner());
    }
  }
}

INSTANTIATE_TEST_CASE_P(AxisAlignedBoxTestGroup, AxisAlignedBoxTest, ::testing::ValuesIn(GetTestParameters()));

class AxisAlignedBoxOverlappingTest : public ::testing::Test {
 public:
  const Vector3 min_corner{-3., -2., -1.};
  const Vector3 max_corner{5., 6., 7.};
  const AxisAlignedBox dut{min_corner, max_corner, kTolerance};
};

// Tests IsBoxContained method on another AxisAlignedBox.
TEST_F(AxisAlignedBoxOverlappingTest, IsBoxContained) {
  // Smaller cube in same location. -> It contains it.
  EXPECT_TRUE(dut.IsBoxContained(AxisAlignedBox({-2., -1., 0.}, {4., 5., 6.}, kTolerance)));
  // Larger cube in same location and orientation. -> It doesn't contain it.
  EXPECT_FALSE(dut.IsBoxContained(AxisAlignedBox({-4., -3., -2.}, {6., 7., 8.}, kTolerance)));
  // Same cube in same location and orientation. -> It contains it.
  EXPECT_TRUE(dut.IsBoxContained(AxisAlignedBox(min_corner, max_corner, kTolerance)));
  // Same cube in different location -> It doesn't contain it.
  EXPECT_FALSE(dut.IsBoxContained(
      AxisAlignedBox(min_corner + 15. * Vector3::Ones(), max_corner + 15. * Vector3::Ones(), kTolerance)));
}

// Tests IsBoxIntersected method.
TEST_F(AxisAlignedBoxOverlappingTest, IsBoxIntersected) {
  // Smaller cube in same location and orientation. -> It contains it so there is intersection.
  EXPECT_TRUE(dut.IsBoxIntersected(AxisAlignedBox({-2., -1., 0.}, {4., 5., 6.}, kTolerance)));
  // Larger cube in same location and orientation. -> Intersects.
  EXPECT_TRUE(dut.IsBoxIntersected(AxisAlignedBox({-4., -3., -2.}, {6., 7., 8.}, kTolerance)));
  // Same cube in different location. -> It doesn't intersect.
  EXPECT_FALSE(dut.IsBoxIntersected(
      AxisAlignedBox(min_corner + 15. * Vector3::Ones(), max_corner + 15. * Vector3::Ones(), kTolerance)));
  // Same size cube but slightly different center. -> Intersects.
  EXPECT_TRUE(
      dut.IsBoxIntersected(AxisAlignedBox(min_corner + Vector3::Ones(), max_corner + Vector3::Ones(), kTolerance)));
}

// Tests from BoundingRegion API.
TEST_F(AxisAlignedBoxOverlappingTest, Overlaps) {
  std::unique_ptr<BoundingRegion<Vector3>> region = std::make_unique<AxisAlignedBox>(dut);
  // Smaller cube in same location and orientation. -> OverlappingType::kContained.
  EXPECT_EQ(OverlappingType::kContained, region->Overlaps(AxisAlignedBox({-2., -1., 0.}, {4., 5., 6.}, kTolerance)));
  // Same cube in same location and orientation. -> OverlappingType::kContained.
  EXPECT_EQ(OverlappingType::kContained, region->Overlaps(AxisAlignedBox(min_corner, max_corner, kTolerance)));
  // Same cube in a different location, no points in common. -> OverlappingType::kDisjointed.
  EXPECT_EQ(OverlappingType::kDisjointed,
            region->Overlaps(
                AxisAlignedBox(min_corner + 15. * Vector3::Ones(), max_corner + 15. * Vector3::Ones(), kTolerance)));
  // Same size cube but slightly different center. -> OverlappingType::kIntersected.
  EXPECT_EQ(OverlappingType::kIntersected,
            region->Overlaps(AxisAlignedBox(min_corner + Vector3::Ones(), max_corner + Vector3::Ones(), kTolerance)));
}

}  // namespace
}  // namespace test
}  // namespace math
}  // namespace maliput
