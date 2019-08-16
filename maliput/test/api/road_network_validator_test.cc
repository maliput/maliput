#include "maliput/api/road_network_validator.h"

#include <exception>

#include <gtest/gtest.h>

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/common/assertion_error.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace api {
namespace test {
namespace {

using rules::DirectionUsageRule;
using rules::LaneSRange;
using rules::PhaseProvider;
using rules::PhaseRingBook;
using rules::RoadRulebook;
using rules::RuleStateProvider;
using rules::SpeedLimitRule;
using rules::SRange;
using rules::TrafficLightBook;

GTEST_TEST(RoadNetworkValidatorTest, RuleCoverageTest) {
  RoadNetwork road_network(test::CreateOneLaneRoadGeometry(), test::CreateRoadRulebook(),
                           test::CreateTrafficLightBook(), test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRuleStateProvider(), test::CreatePhaseProvider());

  RoadNetworkValidatorOptions options{
      true /* check_direction_usage_rule_coverage */,
      false /* check_road_geometry_invariants */,
      false /* check_road_geometry_hierarchy */};
  EXPECT_THROW(ValidateRoadNetwork(road_network, options),
               maliput::common::assertion_error);

  options.check_direction_usage_rule_coverage = false;
  EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
}

// Tests RoadGeometry hierarchy by evaluating partially incomplete RoadGeometries
class RoadGeometryHierarchyTest : public ::testing::TestWithParam<RoadGeometryBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  RoadGeometryBuildFlags build_flags_;
};

std::vector<RoadGeometryBuildFlags> HierarchyTestParameters() {
  return {
    // Throws because of missing Junction.
    RoadGeometryBuildFlags{false, false, false, false, false, true},
    // Throws because of missing Segment in Junction.
    RoadGeometryBuildFlags{true, false, false, false, false, true},
    // Throws because of missing Lane in Segment.
    RoadGeometryBuildFlags{true, true, false, false, false, true},
    // Throws because of missing BranchPoint.
    RoadGeometryBuildFlags{true, true, true, false, false, true},
    // Throws because of missing LaneEndSet in BranchPoint.
    RoadGeometryBuildFlags{true, true, true, true, false, true},
    // Does not throw, complete RoadGeometry.
    RoadGeometryBuildFlags{true, true, true, true, true, false},
  };
}

TEST_P(RoadGeometryHierarchyTest, HierarchyTestThrows) {
  RoadNetwork road_network(CreateRoadGeometry(build_flags_), test::CreateRoadRulebook(), test::CreateTrafficLightBook(),
                           test::CreateIntersectionBook(), test::CreatePhaseRingBook(), test::CreateRuleStateProvider(),
                           test::CreatePhaseProvider());

  const RoadNetworkValidatorOptions options{
      false /* check_direction_usage_rule_coverage */, false /* check_road_geometry_invariants */,
      true /* check_road_geometry_hierarchy */};

  if (build_flags_.expects_throw) {
    EXPECT_THROW({ ValidateRoadNetwork(road_network, options); }, maliput::common::assertion_error);
  } else {
    ValidateRoadNetwork(road_network, options);
    EXPECT_NO_THROW({ ValidateRoadNetwork(road_network, options); });
  }
}

INSTANTIATE_TEST_CASE_P(RoadGeometryHierarchyTestGroup, RoadGeometryHierarchyTest,
                        ::testing::ValuesIn(HierarchyTestParameters()));

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
