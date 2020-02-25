#include "maliput-utilities/segment_analysis.h"

#include <memory>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "maliput/api/segment.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace utility {
namespace {

GTEST_TEST(MockSegmentAnalysisAnalyzeConfluentSegments, BasicOperation) {
  const std::unique_ptr<const api::RoadGeometry> rg = api::test::CreateOneLaneRoadGeometry();

  const std::vector<std::unordered_set<const api::Segment*>> groups = AnalyzeConfluentSegments(rg.get());

  EXPECT_TRUE(groups.empty());
}

}  // anonymous namespace
}  // namespace utility
}  // namespace maliput
