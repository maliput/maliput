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
using rules::PhaseProvider;
using rules::PhaseRingBook;
using rules::RightOfWayRuleStateProvider;
using rules::RoadRulebook;
using rules::SpeedLimitRule;
using rules::TrafficLightBook;

GTEST_TEST(RoadNetworkValidatorTest, RuleCoverageTest) {
  RoadNetwork road_network(test::CreateOneLaneRoadGeometry(), test::CreateRoadRulebook(),
                           test::CreateTrafficLightBook(), test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  RoadNetworkValidatorOptions options{true /* check_direction_usage_rule_coverage */,
                                      false /* check_road_geometry_invariants */,
                                      false /* check_road_geometry_hierarchy */, false /* check_related_bulb_groups */,
                                      false /* check_contiguity_rule_zones*/};
  EXPECT_THROW(ValidateRoadNetwork(road_network, options), common::assertion_error);

  options.check_direction_usage_rule_coverage = false;
  EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
}

// Tests RoadGeometry hierarchy by evaluating partially incomplete RoadGeometries.
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
                           test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  const RoadNetworkValidatorOptions options{
      false /* check_direction_usage_rule_coverage */, false /* check_road_geometry_invariants */,
      true /* check_road_geometry_hierarchy */, false /* check_related_bulb_groups */,
      false /* check_contiguity_rule_zones*/};
  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), common::assertion_error);
  } else {
    ValidateRoadNetwork(road_network, options);
    EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
  }
}

INSTANTIATE_TEST_CASE_P(RoadGeometryHierarchyTestGroup, RoadGeometryHierarchyTest,
                        ::testing::ValuesIn(HierarchyTestParameters()));

struct RelatedBulbGroupTestParam {
  RoadRulebookBuildFlags rulebook_build_flags{};
  TrafficLightBookBuildFlags traffic_light_book_build_flags{};
  bool expects_throw{false};
};

// Tests RoadGeometry hierarchy by evaluating inconsistent TrafficLights and
// BulbGroup ID references in RightOfWayRules.
class RelatedBulbGroupsTest : public ::testing::TestWithParam<RelatedBulbGroupTestParam> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  RelatedBulbGroupTestParam build_flags_;
};

// Returns a vector of RelatedBulbGroupTestParams with test cases.
std::vector<RelatedBulbGroupTestParam> RelatedBulbGroupsTestParameters() {
  return {
      // Does not throw because missing RightOfWayRule to evaluate.
      RelatedBulbGroupTestParam{{false, {false}, false, false}, {false, {false, false}}, false},
      // Does not throw because of missing RelatedBulbGroups.
      RelatedBulbGroupTestParam{{true, {false}, false, false}, {false, {false, false}}, false},
      // Throws because of empty TrafficLightBook.
      RelatedBulbGroupTestParam{{true, {true}, false, false}, {false, {false, false}}, true},
      // Throws because of missing TrafficLight in TrafficLightBook.
      RelatedBulbGroupTestParam{{true, {true}, false, false}, {true, {true, false}}, true},
      // Throws because of missing BulbGroup in TrafficLight.
      RelatedBulbGroupTestParam{{true, {true}, false, false}, {true, {false, true}}, true},
      // Does not throw because of correct structure.
      RelatedBulbGroupTestParam{{true, {true}, false, false}, {true, {false, false}}, false},
  };
}

TEST_P(RelatedBulbGroupsTest, ChecksRelatedBulGroupsRelation) {
  RoadNetwork road_network(CreateRoadGeometry(), test::CreateRoadRulebook(build_flags_.rulebook_build_flags),
                           test::CreateTrafficLightBook(build_flags_.traffic_light_book_build_flags),
                           test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  const RoadNetworkValidatorOptions options{
      false /* check_direction_usage_rule_coverage */, false /* check_road_geometry_invariants */,
      false /* check_road_geometry_hierarchy */, true /* check_related_bulb_groups */,
      false /* check_contiguity_rule_zones*/};
  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), common::assertion_error);
  } else {
    ValidateRoadNetwork(road_network, options);
    EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
  }
}

INSTANTIATE_TEST_CASE_P(RelatedBulbGroupsTestGroup, RelatedBulbGroupsTest,
                        ::testing::ValuesIn(RelatedBulbGroupsTestParameters()));

// Tests the G1 contiguity in Rule's LaneSRoutes.
class ContiguityBetweenLanesTest : public ::testing::TestWithParam<RoadNetworkContiguityBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  RoadNetworkContiguityBuildFlags build_flags_;
};

std::vector<RoadNetworkContiguityBuildFlags> ContiguityTestParameters() {
  // RoadGeometry's angular and linear tolerance.
  // Its value is based on the Cartesian distance between non-contiguous lane endpoints
  // which are created in this test.
  const double linear_tolerance = 1e-3;
  // Its value is based on the angular distance used for non-contiguous lane endpoints
  // which are created in this test.
  const double angular_tolerance = 1e-3;
  return {
      // Without DiscreteValueRule or RangeValueRule.
      // Does not throw.
      // { @
      RoadNetworkContiguityBuildFlags{{false, false, linear_tolerance, angular_tolerance}, {false, false}, false},
      RoadNetworkContiguityBuildFlags{{false, true, linear_tolerance, angular_tolerance}, {false, false}, false},
      RoadNetworkContiguityBuildFlags{{true, false, linear_tolerance, angular_tolerance}, {false, false}, false},
      RoadNetworkContiguityBuildFlags{{true, true, linear_tolerance, angular_tolerance}, {false, false}, false},
      // } @

      // Adding a RangeValueRule.
      // { @
      // Contiguous LaneSRoute with a RangeValueRule.
      RoadNetworkContiguityBuildFlags{{false, false, linear_tolerance, angular_tolerance}, {false, true}, false},
      // Throws because it does not meet the angular tolerance.
      RoadNetworkContiguityBuildFlags{{false, true, linear_tolerance, angular_tolerance}, {false, true}, true},
      // Throws because it does not meet the linear tolerance.
      RoadNetworkContiguityBuildFlags{{true, false, linear_tolerance, angular_tolerance}, {false, true}, true},
      // Throws because it does not meet neither the linear tolerance nor the angular tolerance.
      RoadNetworkContiguityBuildFlags{{true, true, linear_tolerance, angular_tolerance}, {false, true}, true},
      // } @

      // Adding a DiscreteValueRule.
      // { @
      // Contiguous LaneSRoute with a DiscreteValueRule.
      RoadNetworkContiguityBuildFlags{{false, false, linear_tolerance, angular_tolerance}, {true, false}, false},
      // Throws because it does not meet angular tolerance.
      RoadNetworkContiguityBuildFlags{{false, true, linear_tolerance, angular_tolerance}, {true, false}, true},
      // Throws because it does not meet linear tolerance.
      RoadNetworkContiguityBuildFlags{{true, false, linear_tolerance, angular_tolerance}, {true, false}, true},
      // Throws because it does not meet neither the linear tolerance nor the angular tolerance.
      RoadNetworkContiguityBuildFlags{{true, true, linear_tolerance, angular_tolerance}, {true, false}, true},
      // } @

      // Adding DiscreteValueRule and RangeValueRule.
      // { @
      // Contiguous LaneSRoute with a DiscreteValueRule and RangeValueRule.
      RoadNetworkContiguityBuildFlags{{false, false, linear_tolerance, angular_tolerance}, {true, true}, false},
      // Throws because it does not meet angular tolerance.
      RoadNetworkContiguityBuildFlags{{false, true, linear_tolerance, angular_tolerance}, {true, true}, true},
      // Throws because it does not meet linear tolerance.
      RoadNetworkContiguityBuildFlags{{true, false, linear_tolerance, angular_tolerance}, {true, true}, true},
      // Throws because it does not meet neither the linear tolerance nor the angular tolerance.
      RoadNetworkContiguityBuildFlags{{true, true, linear_tolerance, angular_tolerance}, {true, true}, true},
      // } @
  };
}

TEST_P(ContiguityBetweenLanesTest, ChecksContiguityBetweenLanes) {
  RoadNetwork road_network(test::CreateMockContiguousRoadGeometry(build_flags_.rg_contiguity_build_flags),
                           test::CreateMockContiguousRoadRulebook(build_flags_.rulebook_contiguity_build_flags),
                           test::CreateTrafficLightBook(), test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  const RoadNetworkValidatorOptions options{
      false /* check_direction_usage_rule_coverage */, false /* check_road_geometry_invariants */,
      false /* check_road_geometry_hierarchy */, false /* check_related_bulb_groups */,
      true /* check_contiguity_rule_zones*/};

  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), common::assertion_error);
  } else {
    ValidateRoadNetwork(road_network, options);
    EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
  }
}

INSTANTIATE_TEST_CASE_P(ContiguityBetweenLanesTestGroup, ContiguityBetweenLanesTest,
                        ::testing::ValuesIn(ContiguityTestParameters()));

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
