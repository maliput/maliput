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
      RoadGeometryBuildFlags{false, false, false, false, false, true, {}},
      // Throws because of missing Segment in Junction.
      RoadGeometryBuildFlags{true, false, false, false, false, true, {}},
      // Throws because of missing Lane in Segment.
      RoadGeometryBuildFlags{true, true, false, false, false, true, {}},
      // Throws because of missing BranchPoint.
      RoadGeometryBuildFlags{true, true, true, false, false, true, {}},
      // Throws because of missing LaneEndSet in BranchPoint.
      RoadGeometryBuildFlags{true, true, true, true, false, true, {}},
      // Throws because of BranchPointIds are not in RoadGeometry::IdIndex.
      RoadGeometryBuildFlags{true, true, true, true, true, true, {false, true, true, true}},
      // Throws because of JunctionId are not in RoadGeometry::IdIndex.
      RoadGeometryBuildFlags{true, true, true, true, true, true, {true, false, true, true}},
      // Throws because of SegmentId are not in RoadGeometry::IdIndex.
      RoadGeometryBuildFlags{true, true, true, true, true, true, {true, true, true, false}},
      // Throws because of LaneId are not in RoadGeometry::IdIndex.
      RoadGeometryBuildFlags{true, true, true, true, true, true, {true, true, false, true}},
      // Does not throw, complete RoadGeometry.
      RoadGeometryBuildFlags{true, true, true, true, true, false, {true, true, true, true}},
  };
}

TEST_P(RoadGeometryHierarchyTest, HierarchyTestThrows) {
  RoadNetwork road_network(CreateRoadGeometry(build_flags_), test::CreateRoadRulebook(), test::CreateTrafficLightBook(),
                           test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  const RoadNetworkValidatorOptions options{false /* check_direction_usage_rule_coverage */,
                                            false /* check_road_geometry_invariants */,
                                            true /* check_road_geometry_hierarchy */,
                                            false /* check_related_bulb_groups */,
                                            false /* check_contiguity_rule_zones*/,
                                            false /* check_phase_discrete_value_rule_states*/,
                                            false /* check_phase_bulb_states */};
  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), common::assertion_error);
  } else {
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

  const RoadNetworkValidatorOptions options{false /* check_direction_usage_rule_coverage */,
                                            false /* check_road_geometry_invariants */,
                                            false /* check_road_geometry_hierarchy */,
                                            true /* check_related_bulb_groups */,
                                            false /* check_contiguity_rule_zones*/,
                                            false /* check_phase_discrete_value_rule_states*/,
                                            false /* check_phase_bulb_states */};
  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), common::assertion_error);
  } else {
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

  const RoadNetworkValidatorOptions options{false /* check_direction_usage_rule_coverage */,
                                            false /* check_road_geometry_invariants */,
                                            false /* check_road_geometry_hierarchy */,
                                            false /* check_related_bulb_groups */,
                                            true /* check_contiguity_rule_zones*/,
                                            false /* check_phase_discrete_value_rule_states*/,
                                            false /* check_phase_bulb_states */};

  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), common::assertion_error);
  } else {
    EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
  }
}

INSTANTIATE_TEST_CASE_P(ContiguityBetweenLanesTestGroup, ContiguityBetweenLanesTest,
                        ::testing::ValuesIn(ContiguityTestParameters()));

// Holds rules::Phase consistency across the RoadNetwork.
struct PhaseChecksBuildFlags {
  RoadRulebookBuildFlags rulebook_build_flags{};
  PhaseBuildFlags phase_build_flags{};
  bool expects_throw{false};
};

// Evaluates rules::Phase::discrete_value_rule_states() consistency.
class PhaseDiscreteValueRuleStatesTest : public ::testing::TestWithParam<PhaseChecksBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  PhaseChecksBuildFlags build_flags_;
};

std::vector<PhaseChecksBuildFlags> PhaseDiscreteValueRuleStatesTestParameters() {
  return {
      // Throws because of unknown rules::Rule::Id.
      PhaseChecksBuildFlags{{false, {}, false, false, true, false}, {true, false, false, false}, true},
      // Throws because of unknown rules::DiscreteValueRule::DiscreteValue.
      PhaseChecksBuildFlags{{false, {}, false, false, true, false}, {false, true, false, false}, true},
      // Expects no throw because of correct construction.
      PhaseChecksBuildFlags{{false, {}, false, false, true, false}, {false, false, false, false}, false},
  };
}

TEST_P(PhaseDiscreteValueRuleStatesTest, ChecksPhaseDiscreteValueRuleStates) {
  RoadNetwork road_network(test::CreateRoadGeometry(), test::CreateRoadRulebook(build_flags_.rulebook_build_flags),
                           test::CreateTrafficLightBook(), test::CreateIntersectionBook(),
                           test::CreatePhaseRingBook(build_flags_.phase_build_flags),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  const RoadNetworkValidatorOptions options{false /* check_direction_usage_rule_coverage */,
                                            false /* check_road_geometry_invariants */,
                                            false /* check_road_geometry_hierarchy */,
                                            false /* check_related_bulb_groups */,
                                            false /* check_contiguity_rule_zones */,
                                            true /* check_phase_discrete_value_rule_states */,
                                            false /* check_phase_bulb_states */};

  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), std::exception);
  } else {
    EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
  }
}

INSTANTIATE_TEST_CASE_P(PhaseDiscreteValueRuleStatesTestGroup, PhaseDiscreteValueRuleStatesTest,
                        ::testing::ValuesIn(PhaseDiscreteValueRuleStatesTestParameters()));

// Evaluates rules::Phase::bulb_states() consistency.
class PhaseBulbStatesTest : public ::testing::TestWithParam<PhaseChecksBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  PhaseChecksBuildFlags build_flags_;
};

std::vector<PhaseChecksBuildFlags> PhaseBulbStatesTestParameters() {
  return {
      // Throws because of unknown rules::UniqueBulbId.
      PhaseChecksBuildFlags{{false, {}, false, false, true, false}, {false, false, true, false}, true},
      // Throws because of unknown rules::BulbState.
      PhaseChecksBuildFlags{{false, {}, false, false, true, false}, {false, false, false, true}, true},
      // Expects no throw because of correct construction.
      PhaseChecksBuildFlags{{false, {}, false, false, true, false}, {false, false, false, false}, false},
  };
}

TEST_P(PhaseBulbStatesTest, ChecksPhaseBulbStates) {
  RoadNetwork road_network(test::CreateRoadGeometry(), test::CreateRoadRulebook(build_flags_.rulebook_build_flags),
                           test::CreateTrafficLightBook(TrafficLightBookBuildFlags{true, {}}),
                           test::CreateIntersectionBook(), test::CreatePhaseRingBook(build_flags_.phase_build_flags),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  const RoadNetworkValidatorOptions options{false /* check_direction_usage_rule_coverage */,
                                            false /* check_road_geometry_invariants */,
                                            false /* check_road_geometry_hierarchy */,
                                            false /* check_related_bulb_groups */,
                                            false /* check_contiguity_rule_zones */,
                                            false /* check_phase_discrete_value_rule_states */,
                                            true /* check_phase_bulb_states */};

  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), std::exception);
  } else {
    EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
  }
}

INSTANTIATE_TEST_CASE_P(PhaseBulbStatesTestGroup, PhaseBulbStatesTest,
                        ::testing::ValuesIn(PhaseBulbStatesTestParameters()));

// Holds the Rulebook's rule construction flags for the test and whether or not
// it must throw.
struct RelatedRulesBuildFlags {
  RoadRulebookRelatedRulesBuildFlags rulebook_build_flags{};
  bool expects_throw{false};
};

// Evaluates whether or not the validation detects inconsistent RelatedRules in
// rules::Rule::States for both rules::DiscreteValueRule and
// rules::RaneValueRule.
class RelatedRulesTest : public ::testing::TestWithParam<RelatedRulesBuildFlags> {
 protected:
  void SetUp() override { build_flags_ = GetParam(); }

  RelatedRulesBuildFlags build_flags_{};
};

// Returns a vector of test configurations to create different tests.
std::vector<RelatedRulesBuildFlags> RelatedRulesTestParameters() {
  return {
      // Throws because of inconsistent RelatedRule in rules::DiscreteValueRule.
      RelatedRulesBuildFlags{{{false, {}, false, false, true, true}, false, true}, true},
      // Throws because of inconsistent RelatedRule in rules::RangeValueRule.
      RelatedRulesBuildFlags{{{false, {}, false, false, true, true}, true, false}, true},
      // Expects no throw because of correct construction.
      RelatedRulesBuildFlags{{{false, {}, false, false, true, true}, true, true}, false},
  };
}

TEST_P(RelatedRulesTest, ChecksRelatedRules) {
  RoadNetwork road_network(test::CreateRoadGeometry(), test::CreateRoadRulebook(build_flags_.rulebook_build_flags),
                           test::CreateTrafficLightBook(), test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRightOfWayRuleStateProvider(), test::CreatePhaseProvider(),
                           test::CreateRuleRegistry(), test::CreateDiscreteValueRuleStateProvider(),
                           test::CreateRangeValueRuleStateProvider());

  const RoadNetworkValidatorOptions options{false /* check_direction_usage_rule_coverage */,
                                            false /* check_road_geometry_invariants */,
                                            false /* check_road_geometry_hierarchy */,
                                            false /* check_related_bulb_groups */,
                                            false /* check_contiguity_rule_zones */,
                                            false /* check_phase_discrete_value_rule_states */,
                                            true /* check_phase_bulb_states */};

  if (build_flags_.expects_throw) {
    EXPECT_THROW(ValidateRoadNetwork(road_network, options), std::exception);
  } else {
    EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
  }
}

INSTANTIATE_TEST_CASE_P(RelatedRulesTestGroup, RelatedRulesTest, ::testing::ValuesIn(RelatedRulesTestParameters()));

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
