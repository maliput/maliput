#include "utilities/generate_string.h"

#include <memory>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/test_utilities/mock.h"

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
