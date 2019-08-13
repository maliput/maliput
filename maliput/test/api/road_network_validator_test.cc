#include "maliput/api/road_network_validator.h"

#include <exception>

#include <gtest/gtest.h>

#include "maliput/api/road_network.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace api {
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

  RoadNetworkValidatorOptions options;
  EXPECT_THROW(ValidateRoadNetwork(road_network, options), std::exception);
  options.check_direction_usage_rule_coverage = false;
  EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
}

}  // namespace
}  // namespace api
}  // namespace maliput
