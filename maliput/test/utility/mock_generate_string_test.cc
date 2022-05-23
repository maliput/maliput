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

#include "maliput/api/road_geometry.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/utility/generate_string.h"

namespace maliput {
namespace utility {
namespace test {
namespace {

class MockGenerateStringTest : public ::testing::Test {
 protected:
  MockGenerateStringTest() { road_geometry_ = api::test::CreateOneLaneRoadGeometry(); }

  std::unique_ptr<const api::RoadGeometry> road_geometry_;
  GenerateStringOptions options_;
};

TEST_F(MockGenerateStringTest, Nothing) { EXPECT_TRUE(GenerateString(*road_geometry_, options_).empty()); }

TEST_F(MockGenerateStringTest, RoadGeometeryJunctionsSegmentsLanes) {
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_segment_ids = true;
  options_.include_lane_ids = true;
  EXPECT_EQ("mock", GenerateString(*road_geometry_, options_));
}

TEST_F(MockGenerateStringTest, RoadGeometeryJunctionsSegments) {
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_segment_ids = true;
  EXPECT_EQ("mock", GenerateString(*road_geometry_, options_));
}

TEST_F(MockGenerateStringTest, RoadGeometeryJunctionsLanes) {
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_lane_ids = true;
  EXPECT_EQ("mock", GenerateString(*road_geometry_, options_));
}

TEST_F(MockGenerateStringTest, RoadGeometeryLanes) {
  options_.include_road_geometry_id = true;
  options_.include_lane_ids = true;
  EXPECT_EQ("mock", GenerateString(*road_geometry_, options_));
}

TEST_F(MockGenerateStringTest, Lanes) {
  options_.include_lane_ids = true;
  EXPECT_TRUE(GenerateString(*road_geometry_, options_).empty());
}

TEST_F(MockGenerateStringTest, LanesWithDetails) {
  options_.include_lane_ids = true;
  options_.include_lane_details = true;
  EXPECT_TRUE(GenerateString(*road_geometry_, options_).empty());
}

TEST_F(MockGenerateStringTest, WithLabels) {
  options_.include_type_labels = true;
  options_.include_road_geometry_id = true;
  options_.include_junction_ids = true;
  options_.include_segment_ids = true;
  options_.include_lane_ids = true;
  options_.include_lane_details = true;
  EXPECT_EQ("geometry: mock", GenerateString(*road_geometry_, options_));
}

}  // anonymous namespace
}  // namespace test
}  // namespace utility
}  // namespace maliput
