#include "utilities/segment_analysis.h"

#include <memory>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "maliput/api/segment.h"
#include "maliput/test_utilities/mock.h"

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
