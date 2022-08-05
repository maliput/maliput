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
#include "maliput/math/bounding_box.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput/math/overlapping_type.h"

namespace maliput {
namespace math {
namespace test {
namespace {

static constexpr double kTolerance{1e-12};

TEST(BoundingBox, Constructors) {
  // Throws due to a negative tolerance value.
  EXPECT_THROW(
      BoundingBox({1., 2., 3.} /* position */, {2., 2., 2.} /* box_size */, {0., 0., 0.} /* rpy */, -1 /* tolerance */),
      maliput::common::assertion_error);
  // It doesn't throw.
  EXPECT_NO_THROW(BoundingBox({1., 2., 3.} /* position */, {2., 2., 2.} /* box_size */, {0., 0., 0.} /* rpy */,
                              1e-12 /* tolerance */););
}

struct ExpectedPositionContainsResults {
  Vector3 position{};
  bool contains_position{};
};

struct BoundingBoxCase {
  Vector3 position{};
  Vector3 box_size{};
  RollPitchYaw rpy{};

  std::vector<ExpectedPositionContainsResults> expected_position_contains_results{};
  std::vector<Vector3> expected_vertices_result{};
};

std::vector<BoundingBoxCase> GetTestParameters() {
  return {
      // Centered at origin, no rotation.
      BoundingBoxCase{
          {0., 0., 0.} /* position */,
          {2., 2., 2.} /* box_size */,
          {0., 0., 0.} /* rpy */,
          {
              {{1., 1., 1}, true},   /* Right in a vertex */
              {{-1., 1., 1}, true},  /* Right in a vertex */
              {{1., -1., 1}, true},  /* Right in a vertex */
              {{1., 1., -1}, true},  /* Right in a vertex */
              {{2., 1., 1.}, false}, /*A bit of on x*/
              {{1., 2., 1.}, false}, /*A bit of on y*/
              {{1., 1., 2.}, false}, /*A bit of on z*/
          },
          {{1., 1., 1.},
           {-1., 1., 1.},
           {1., -1., 1.},
           {1., 1., -1.},
           {-1., -1., 1.},
           {-1., 1., -1.},
           {1., -1., -1.},
           {-1., -1., -1.}} /* vertices */
      },
      // Centered away from origin, no rotation.
      BoundingBoxCase{
          {10., 10., 10.} /* position */,
          {2., 2., 2.} /* box_size */,
          {0., 0., 0.} /* rpy */,
          {
              {{11., 11., 11}, true},   /* Right in a vertex */
              {{9., 11., 11}, true},    /* Right in a vertex */
              {{11., 9., 11}, true},    /* Right in a vertex */
              {{11., 11., 9}, true},    /* Right in a vertex */
              {{12., 11., 11.}, false}, /*A bit of on x*/
              {{11., 12., 11.}, false}, /*A bit of on y*/
              {{11., 11., 12.}, false}, /*A bit of on z*/
          },
          {{11., 11., 11.},
           {9., 11., 11.},
           {11., 9., 11.},
           {11., 11., 9.},
           {9., 9., 11.},
           {9., 11., 9.},
           {11., 9., 9.},
           {9., 9., 9.}} /* vertices */
      },
      // Centered at origin, 45 degree yaw rotation.
      BoundingBoxCase{
          {0., 0., 0.} /* position */,
          {2., 2., 2.} /* box_size */,
          {0., 0., M_PI_4} /* rpy */,
          {
              {{sqrt(2.), 0., 1}, true},   /* Right in a vertex */
              {{-sqrt(2.), 0., 1}, true},  /* Right in a vertex */
              {{sqrt(2.), 0., -1}, true},  /* Right in a vertex */
              {{-sqrt(2.), 0., -1}, true}, /* Right in a vertex */
              {{0., sqrt(2.), 1}, true},   /* Right in a vertex */
              {{0., -sqrt(2.), 1}, true},  /* Right in a vertex */
              {{0., sqrt(2.), -1}, true},  /* Right in a vertex */
              {{0., -sqrt(2.), -1}, true}, /* Right in a vertex */
              {{1., 1., 1}, false},        /* Right in a non-rotated */
              {{-1., 1., 1}, false},       /* Right in a non-rotated */
              {{1., -1., 1}, false},       /* Right in a non-rotated */
              {{1., 1., -1}, false},       /* Right in a non-rotated */
          },
          {{sqrt(2.), 0., 1},
           {-sqrt(2.), 0., 1},
           {sqrt(2.), 0., -1},
           {-sqrt(2.), 0., -1},
           {0., sqrt(2.), 1},
           {0., -sqrt(2.), 1},
           {0., sqrt(2.), -1},
           {0., -sqrt(2.), -1}} /* vertices */
      },
      // Centered at origin, 45 degree pitch rotation.
      BoundingBoxCase{
          {0., 0., 0.} /* position */,
          {2., 2., 2.} /* box_size */,
          {0., M_PI_4, 0.} /* rpy */,
          {
              {{0., 1., sqrt(2.)}, true},   /* Right in a vertex */
              {{0., 1., -sqrt(2.)}, true},  /* Right in a vertex */
              {{0., -1., sqrt(2.)}, true},  /* Right in a vertex */
              {{0., -1., -sqrt(2.)}, true}, /* Right in a vertex */
              {{sqrt(2.), 1., 0.}, true},   /* Right in a vertex */
              {{sqrt(2.), -1., 0.}, true},  /* Right in a vertex */
              {{-sqrt(2.), 1., 0.}, true},  /* Right in a vertex */
              {{-sqrt(2.), -1., 0.}, true}, /* Right in a vertex */
              {{1., 1., 1}, false},         /* Right in a non-rotated */
              {{-1., 1., 1}, false},        /* Right in a non-rotated */
              {{1., -1., 1}, false},        /* Right in a non-rotated */
              {{1., 1., -1}, false},        /* Right in a non-rotated */
          },
          {
              {0., 1., sqrt(2.)},
              {0., 1., -sqrt(2.)},
              {0., -1., sqrt(2.)},
              {0., -1., -sqrt(2.)},
              {sqrt(2.), 1., 0.},
              {sqrt(2.), -1., 0.},
              {-sqrt(2.), 1., 0.},
              {-sqrt(2.), -1., 0.},
          } /* vertices */
      },
      // Centered at origin, 45 degree roll rotation.
      BoundingBoxCase{
          {0., 0., 0.} /* position */,
          {2., 2., 2.} /* box_size */,
          {M_PI_4, 0., 0.} /* rpy */,
          {
              {{1., 0., sqrt(2.)}, true},   /* Right in a vertex */
              {{-1., 0., sqrt(2.)}, true},  /* Right in a vertex */
              {{1., 0., -sqrt(2.)}, true},  /* Right in a vertex */
              {{-1., 0., -sqrt(2.)}, true}, /* Right in a vertex */
              {{1., sqrt(2.), 0.}, true},   /* Right in a vertex */
              {{1., -sqrt(2.), 0.}, true},  /* Right in a vertex */
              {{-1., sqrt(2.), 0.}, true},  /* Right in a vertex */
              {{-1., -sqrt(2.), 0.}, true}, /* Right in a vertex */
              {{1., 1., 1}, false},         /* Right in a non-rotated */
              {{-1., 1., 1}, false},        /* Right in non-rotated */
              {{1., -1., 1}, false},        /* Right in non-rotated */
              {{1., 1., -1}, false},        /* Right in non-rotated */
          },
          {
              {1., 0., sqrt(2.)},
              {-1., 0., sqrt(2.)},
              {1., 0., -sqrt(2.)},
              {-1., 0., -sqrt(2.)},
              {1., sqrt(2.), 0.},
              {1., -sqrt(2.), 0.},
              {-1., sqrt(2.), 0.},
              {-1., -sqrt(2.), 0.},
          } /* vertices */
      },
  };
}

class BoundingBoxTest : public ::testing::TestWithParam<BoundingBoxCase> {
 public:
  void SetUp() override {}
  BoundingBoxCase case_ = GetParam();
};

TEST_P(BoundingBoxTest, ContainsPosition) {
  BoundingBox dut{case_.position, case_.box_size, case_.rpy, kTolerance};

  for (const auto expected_result : case_.expected_position_contains_results) {
    EXPECT_EQ(expected_result.contains_position, dut.Contains(expected_result.position))
        << "Expected contains value: " + std::string(expected_result.contains_position ? "True" : "False") +
               " at position: " + expected_result.position.to_str();
  }
}

TEST_P(BoundingBoxTest, GetVertices) {
  BoundingBox dut{case_.position, case_.box_size, case_.rpy, kTolerance};
  const auto vertices = dut.get_vertices();
  for (const auto& vertex : vertices) {
    const auto expected_vertex_itr =
        std::find_if(case_.expected_vertices_result.begin(), case_.expected_vertices_result.end(),
                     [&vertex](const auto& expected_vertex) {
                       return std::abs(vertex.x() - expected_vertex.x()) < kTolerance &&
                              std::abs(vertex.y() - expected_vertex.y()) < kTolerance &&
                              std::abs(vertex.z() - expected_vertex.z()) < kTolerance;
                     });
    EXPECT_NE(expected_vertex_itr, case_.expected_vertices_result.end())
        << "Vertex: " << vertex.to_str() << " doesn't match an expected vertex.";
  }
}

TEST_P(BoundingBoxTest, GetBoxSize) {
  BoundingBox dut{case_.position, case_.box_size, case_.rpy, kTolerance};
  EXPECT_EQ(case_.box_size, dut.box_size());
}

TEST_P(BoundingBoxTest, GetPosition) {
  BoundingBox dut{case_.position, case_.box_size, case_.rpy, kTolerance};
  EXPECT_EQ(case_.position, dut.position());
}

TEST_P(BoundingBoxTest, GetOrientation) {
  BoundingBox dut{case_.position, case_.box_size, case_.rpy, kTolerance};
  EXPECT_EQ(case_.rpy.vector(), dut.get_orientation().vector());
}

INSTANTIATE_TEST_CASE_P(BoundingBoxTestGroup, BoundingBoxTest, ::testing::ValuesIn(GetTestParameters()));

class BoundingBoxOverlappingTest : public ::testing::Test {
 public:
  const Vector3 position{1., 2., 3.};
  const Vector3 box_size{4., 4., 4.};
  const RollPitchYaw rpy{M_PI_4, M_PI_4, M_PI_4};
  const BoundingBox dut{position, box_size, rpy, kTolerance};
};

// Tests IsBoxContained method on another BoundingBox.
// This method is implemented using `get_vertices` method and `Contains` method for positions, which were already
// tested.
TEST_F(BoundingBoxOverlappingTest, IsBoxContained) {
  // Smaller cube in same location and orientation. -> It contains it.
  EXPECT_TRUE(dut.IsBoxContained(BoundingBox(position, {3., 3., 3.}, rpy, kTolerance)));
  // Larger cube in same location and orientation. -> It doesn't contain it.
  EXPECT_FALSE(dut.IsBoxContained(BoundingBox(position, {5., 5., 5.}, rpy, kTolerance)));
  // Same cube in same location and orientation. -> It contains it.
  EXPECT_TRUE(dut.IsBoxContained(BoundingBox(position, box_size, rpy, kTolerance)));
  // Same cube in different location and same orientation. -> It doesn't contain it.
  EXPECT_FALSE(dut.IsBoxContained(BoundingBox({-1, -2., -3.}, box_size, rpy, kTolerance)));
  // Same cube in same location and different orientation. -> It doesn't contain it.
  EXPECT_FALSE(dut.IsBoxContained(BoundingBox(position, box_size, {0., 0., 0.}, kTolerance)));
}

// Tests IsBoxIntersected method. The method is based on Drake's implementation of
// drake::geometry::internal::BoxesOverlap() method. See
// https://github.com/RobotLocomotion/drake/blob/master/geometry/proximity/boxes_overlap.cc
TEST_F(BoundingBoxOverlappingTest, IsBoxIntersected) {
  // Smaller cube in same location and orientation. -> It contains it so there is intersection.
  EXPECT_TRUE(dut.IsBoxIntersected(BoundingBox(position, {3., 3., 3.}, rpy, kTolerance)));
  // Larger cube in same location and orientation. -> Intersects.
  EXPECT_TRUE(dut.IsBoxIntersected(BoundingBox(position, {5., 5., 5.}, rpy, kTolerance)));
  // Same cube in same location and different orientation. -> Intersects.
  EXPECT_TRUE(dut.IsBoxIntersected(BoundingBox(position, box_size, {0., 0., 0.}, kTolerance)));
  // Same cube in different location. -> It doesn't intersect.
  EXPECT_FALSE(dut.IsBoxIntersected(BoundingBox({-1, -2., -3.}, box_size, rpy, kTolerance)));
}

TEST_F(BoundingBoxOverlappingTest, Overlaps) {
  // Tests from BoundingRegion API.
  std::unique_ptr<BoundingRegion<Vector3>> region = std::make_unique<BoundingBox>(dut);
  // Smaller cube in same location and orientation. -> OverlappingType::kContained.
  EXPECT_EQ(OverlappingType::kContained, region->Overlaps(BoundingBox(position, {3., 3., 3.}, rpy, kTolerance)));
  // Same cube in same location and orientation. -> OverlappingType::kContained.
  EXPECT_EQ(OverlappingType::kContained, region->Overlaps(BoundingBox(position, box_size, rpy, kTolerance)));
  // Same cube in a different location, no points in common. -> OverlappingType::kDisjointed.
  EXPECT_EQ(OverlappingType::kDisjointed, region->Overlaps(BoundingBox({-1, -2., -3.}, box_size, rpy, kTolerance)));
  // Same cube in same location and different orientation. -> OverlappingType::kIntersected.
  EXPECT_EQ(OverlappingType::kIntersected, region->Overlaps(BoundingBox(position, box_size, {0., 0., 0.}, kTolerance)));
}

}  // namespace
}  // namespace test
}  // namespace math
}  // namespace maliput
