#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/vehicle_motorization_rule.h"
#include "maliput/api/rules/vehicle_usage_rule.h"

#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"

#include "maliput/base/manual_rulebook.h"

#include "multilane/builder.h"
#include "multilane/loader.h"


using maliput::api::LaneId;
using maliput::api::RoadGeometry;
using maliput::api::rules::LaneSRange;
using maliput::api::rules::RoadRulebook;
using maliput::api::rules::RuleBase;
using maliput::api::rules::RuleGroup;
using maliput::api::rules::RuleState;
using maliput::api::rules::SRange;
using maliput::api::rules::VehicleMotorizationRule;
using maliput::api::rules::VehicleMotorizationState;
using maliput::api::rules::VehicleMotorizationType;
using maliput::api::rules::VehicleUsageRule;
using maliput::api::rules::VehicleUsageState;
using maliput::api::rules::VehicleUsageType;


namespace maliput {
namespace test {
namespace {

// The following string describes a single segment road with six lanes. Each
// will be thought to describe the following RoadGeometry and Usage.
//
//  +-------- Sidewalk >---------+  ==> VehiclesNotAllowed
//  |-------- Bike lane >--------|  ==> VehiclesAllowed ==> NonMotorizedVehicles
//  |-------- Driving lane >-----|  ==> VehiclesAllowed ==> MotorizedVehicles
//  |-------- Driving lane >-----|  ==> VehiclesAllowed ==> MotorizedVehicles
//  |-------- Bike lane >--------|  ==> VehiclesAllowed ==> NonMotorizedVehicles
//  +-------- Sidewalk >---------+  ==> VehiclesNotAllowed
//
const char* kMultilaneYaml =
R"R(maliput_multilane_builder:
  id: "full_street"
  lane_width: 3.5
  left_shoulder: 0.1
  right_shoulder: 0.1
  elevation_bounds: [0., 5.]
  scale_length: 1.
  linear_tolerance: 0.01
  angular_tolerance: 0.01
  computation_policy: prefer-accuracy
  points:
    a:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
  connections:
    s1:
      lanes: [6, 0, 0]
      start: ["ref", "points.a.forward"]
      length: 100
      z_end: ["ref", [0, 0, 0, 0]]
)R";

// Builds a LaneSRange that covers the full `lane_id` extension.
//
// @param rg is the RoadGeometry pointer. It must not be nullptr.
// @param lane_id is unique ID of the Lane within `rg`.
LaneSRange BuildLaneRangeFor(const RoadGeometry* rg, const LaneId lane_id) {
  DRAKE_DEMAND(rg != nullptr);
  return LaneSRange(lane_id, SRange(0., rg->ById().GetLane(lane_id)->length()));
}

// Builds a VehicleUsageRule that prohibits vehicles along the whole `lane_id`
// extension.
//
// @param rg is the RoadGeometry pointer. It must not be nullptr.
// @param lane_id is unique ID of the Lane within `rg`.
std::unique_ptr<RuleBase> BuildSidewalkRuleFor(
    const RoadGeometry* rg, const LaneId& lane_id) {
  DRAKE_DEMAND(rg != nullptr);

  std::vector<std::unique_ptr<RuleState>> states;
  states.push_back(std::make_unique<VehicleUsageState>(
      RuleState::Id(std::string("vurs_") + lane_id.string()),
      RuleState::Severity::kStrict,
      VehicleUsageType::VehiclesNotAllowed()));

  return std::make_unique<VehicleUsageRule>(
      RuleBase::Id(std::string("vur_") + lane_id.string()),
      BuildLaneRangeFor(rg, lane_id),
      std::move(states));
}

// Builds a RuleGroup to allow non-motorized vehicles to drive along `lane_id`.
//
// @param rg is the RoadGeometry pointer. It must not be nullptr.
// @param lane_id is unique ID of the Lane within `rg`.
std::unique_ptr<RuleGroup> BuildBikeLaneRoadRuleGroupFor(
    const RoadGeometry* rg, const LaneId& lane_id) {
  DRAKE_DEMAND(rg != nullptr);

  std::vector<std::unique_ptr<RuleBase>> rules;
  {
    std::vector<std::unique_ptr<RuleState>> states;
    states.push_back(std::make_unique<VehicleUsageState>(
        RuleState::Id(std::string("vurs_") + lane_id.string()),
        RuleState::Severity::kStrict,
        VehicleUsageType::VehiclesAllowed()));

    rules.push_back(std::make_unique<VehicleUsageRule>(
        RuleBase::Id(std::string("vur_") + lane_id.string()),
        BuildLaneRangeFor(rg, lane_id),
        std::move(states)));
  }
  {
    std::vector<std::unique_ptr<RuleState>> states;
    states.push_back(std::make_unique<VehicleMotorizationState>(
        RuleState::Id(std::string("vmrs_") + lane_id.string()),
        RuleState::Severity::kStrict,
        VehicleMotorizationType::NonMotorizedVehicles()));
    rules.push_back(std::make_unique<VehicleMotorizationRule>(
        RuleBase::Id(std::string("vmr_") + lane_id.string()),
        BuildLaneRangeFor(rg, lane_id),
        std::move(states)));
  }

  return std::make_unique<RuleGroup>(
      RuleGroup::Id(std::string("rg_") + lane_id.string()), std::move(rules));
}

// Builds a RuleGroup to allow motorized vehicles to drive along `lane_id`.
//
// @param rg is the RoadGeometry pointer. It must not be nullptr.
// @param lane_id is unique ID of the Lane within `rg`.
std::unique_ptr<RuleGroup> BuildMotorizedLaneRoadRuleGroupFor(
    const RoadGeometry* rg, const LaneId& lane_id) {
  DRAKE_DEMAND(rg != nullptr);

  std::vector<std::unique_ptr<RuleBase>> rules;
  {
    std::vector<std::unique_ptr<RuleState>> states;
    states.push_back(std::make_unique<VehicleUsageState>(
        RuleState::Id(std::string("vurs_") + lane_id.string()),
        RuleState::Severity::kStrict,
        VehicleUsageType::VehiclesAllowed()));

    rules.push_back(std::make_unique<VehicleUsageRule>(
        RuleBase::Id(std::string("vur_") + lane_id.string()),
        BuildLaneRangeFor(rg, lane_id),
        std::move(states)));
  }
  {
    std::vector<std::unique_ptr<RuleState>> states;
    states.push_back(std::make_unique<VehicleMotorizationState>(
        RuleState::Id(std::string("vmrs_") + lane_id.string()),
        RuleState::Severity::kStrict,
        VehicleMotorizationType::MotorizedVehicles()));
    rules.push_back(std::make_unique<VehicleMotorizationRule>(
        RuleBase::Id(std::string("vmr_") + lane_id.string()),
        BuildLaneRangeFor(rg, lane_id),
        std::move(states)));
  }

  return std::make_unique<RuleGroup>(
      RuleGroup::Id(std::string("rg_") + lane_id.string()), std::move(rules));
}

// Builds a ManualRoadRulebook for `rg` considering previous lane vehicle usage.
//
// @param rg is the RoadGeometry pointer. It must not be nullptr.
std::unique_ptr<RoadRulebook> BuildRoadRulebookFor(
    const RoadGeometry* rg) {
  DRAKE_THROW_UNLESS(rg != nullptr);

  auto rulebook = std::make_unique<maliput::ManualRulebook>();

  // Builds sidewalk rules
  rulebook->AddRule(BuildSidewalkRuleFor(rg, LaneId("l:s1_0")));
  rulebook->AddRule(BuildSidewalkRuleFor(rg, LaneId("l:s1_5")));

  // Builds bikelane rules
  rulebook->AddRuleGroup(BuildBikeLaneRoadRuleGroupFor(rg, LaneId("l:s1_1")));
  rulebook->AddRuleGroup(BuildBikeLaneRoadRuleGroupFor(rg, LaneId("l:s1_4")));

  // Builds motorized lane rules
  rulebook->AddRuleGroup(
      BuildMotorizedLaneRoadRuleGroupFor(rg, LaneId("l:s1_2")));
  rulebook->AddRuleGroup(
      BuildMotorizedLaneRoadRuleGroupFor(rg, LaneId("l:s1_3")));

  return std::move(rulebook);
}


// Evaluates rules and RuleGroups that determine which kind of agent can drive
// along a specific lane.
GTEST_TEST(VehicleRulesAndAgentTest, VehicleRuleQueries) {
  const double kTolerance{0.01};
  const SRange kSRange(10., 20.);

  // Builds the RoadGeometry
  auto rg = maliput::multilane::Load(maliput::multilane::BuilderFactory(),
                                     kMultilaneYaml);
  // Builds the RoadRulebook
  auto rulebook = BuildRoadRulebookFor(rg.get());

  // Queries the RoadRulebook at different lanes and positions to understand if
  // an agent should drive or not through that lane.

  // Fist, it tests sidewalks:
  RoadRulebook::QueryResults query_result;

  query_result = rulebook->FindRules(
      {LaneSRange(LaneId("l:s1_0"), kSRange)}, kTolerance);

  EXPECT_TRUE(query_result.right_of_way.empty());
  EXPECT_TRUE(query_result.speed_limit.empty());
  EXPECT_TRUE(query_result.direction_usage.empty());
  EXPECT_TRUE(query_result.rule_group.empty());
  EXPECT_EQ(query_result.rule.size(), 1);
  EXPECT_EQ(query_result.rule[0]->id(), RuleBase::Id("vur_l:s1_0"));
  EXPECT_EQ(query_result.rule[0]->static_state().type()->string(),
            VehicleUsageType::VehiclesNotAllowed()->string());
  EXPECT_EQ(query_result.rule[0]->static_state().type()->value(),
            VehicleUsageType::VehiclesNotAllowed()->value());

  query_result = rulebook->FindRules(
      {LaneSRange(LaneId("l:s1_5"), SRange(10., 20.))}, kTolerance);

  EXPECT_TRUE(query_result.right_of_way.empty());
  EXPECT_TRUE(query_result.speed_limit.empty());
  EXPECT_TRUE(query_result.direction_usage.empty());
  EXPECT_TRUE(query_result.rule_group.empty());
  EXPECT_EQ(query_result.rule.size(), 1);
  EXPECT_EQ(query_result.rule[0]->id(), RuleBase::Id("vur_l:s1_5"));
  EXPECT_EQ(query_result.rule[0]->static_state().type()->string(),
            VehicleUsageType::VehiclesNotAllowed()->string());
  EXPECT_EQ(query_result.rule[0]->static_state().type()->value(),
            VehicleUsageType::VehiclesNotAllowed()->value());

  // Then, bike lanes:
  query_result = rulebook->FindRules(
      {LaneSRange(LaneId("l:s1_1"), SRange(10., 20.))}, kTolerance);

  EXPECT_TRUE(query_result.right_of_way.empty());
  EXPECT_TRUE(query_result.speed_limit.empty());
  EXPECT_TRUE(query_result.direction_usage.empty());
  EXPECT_TRUE(query_result.rule.empty());
  EXPECT_EQ(query_result.rule_group.size(), 1);
  EXPECT_EQ(query_result.rule_group[0]->id(), RuleGroup::Id("rg_l:s1_1"));
  EXPECT_EQ(query_result.rule_group[0]->size(), 2);
  // Note: rules within a RuleGroup do not need to be in any specific ordering,
  // however, abusing of how it was built, specific types are expected.
  EXPECT_EQ(query_result.rule_group[0]->rule(0)->rule_type(),
            VehicleUsageRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->string(),
      VehicleUsageType::VehiclesAllowed()->string());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->value(),
      VehicleUsageType::VehiclesAllowed()->value());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(1)->rule_type(),
      VehicleMotorizationRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(1)->static_state().type()->string(),
      VehicleMotorizationType::NonMotorizedVehicles()->string());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(1)->static_state().type()->value(),
      VehicleMotorizationType::NonMotorizedVehicles()->value());

  query_result = rulebook->FindRules(
      {LaneSRange(LaneId("l:s1_4"), SRange(10., 20.))}, kTolerance);

  EXPECT_TRUE(query_result.right_of_way.empty());
  EXPECT_TRUE(query_result.speed_limit.empty());
  EXPECT_TRUE(query_result.direction_usage.empty());
  EXPECT_TRUE(query_result.rule.empty());
  EXPECT_EQ(query_result.rule_group.size(), 1);
  EXPECT_EQ(query_result.rule_group[0]->id(), RuleGroup::Id("rg_l:s1_4"));
  EXPECT_EQ(query_result.rule_group[0]->size(), 2);
  // Note: rules within a RuleGroup do not need to be in any specific ordering,
  // however, abusing of how it was built, specific types are expected.
  EXPECT_EQ(query_result.rule_group[0]->rule(0)->rule_type(),
            VehicleUsageRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->string(),
      VehicleUsageType::VehiclesAllowed()->string());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->value(),
      VehicleUsageType::VehiclesAllowed()->value());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(1)->rule_type(),
      VehicleMotorizationRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(1)->static_state().type()->string(),
      VehicleMotorizationType::NonMotorizedVehicles()->string());
  EXPECT_EQ(query_result.rule_group[0]->rule(1)->static_state().type()->value(),
            VehicleMotorizationType::NonMotorizedVehicles()->value());

  // Finally, driving motorized lanes:
  query_result = rulebook->FindRules(
      {LaneSRange(LaneId("l:s1_2"), SRange(10., 20.))}, kTolerance);

  EXPECT_TRUE(query_result.right_of_way.empty());
  EXPECT_TRUE(query_result.speed_limit.empty());
  EXPECT_TRUE(query_result.direction_usage.empty());
  EXPECT_TRUE(query_result.rule.empty());
  EXPECT_EQ(query_result.rule_group.size(), 1);
  EXPECT_EQ(query_result.rule_group[0]->id(), RuleGroup::Id("rg_l:s1_2"));
  EXPECT_EQ(query_result.rule_group[0]->size(), 2);
  // Note: rules within a RuleGroup do not need to be in any specific ordering,
  // however, abusing of how it was built, specific types are expected.
  EXPECT_EQ(query_result.rule_group[0]->rule(0)->rule_type(),
            VehicleUsageRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->string(),
      VehicleUsageType::VehiclesAllowed()->string());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->value(),
      VehicleUsageType::VehiclesAllowed()->value());
  EXPECT_EQ(query_result.rule_group[0]->rule(1)->rule_type(),
            VehicleMotorizationRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(1)->static_state().type()->string(),
      VehicleMotorizationType::MotorizedVehicles()->string());
  EXPECT_EQ(query_result.rule_group[0]->rule(1)->static_state().type()->value(),
            VehicleMotorizationType::MotorizedVehicles()->value());

  query_result = rulebook->FindRules(
      {LaneSRange(LaneId("l:s1_3"), SRange(10., 20.))}, kTolerance);

  EXPECT_TRUE(query_result.right_of_way.empty());
  EXPECT_TRUE(query_result.speed_limit.empty());
  EXPECT_TRUE(query_result.direction_usage.empty());
  EXPECT_TRUE(query_result.rule.empty());
  EXPECT_EQ(query_result.rule_group.size(), 1);
  EXPECT_EQ(query_result.rule_group[0]->id(), RuleGroup::Id("rg_l:s1_3"));
  EXPECT_EQ(query_result.rule_group[0]->size(), 2);
  // Note: rules within a RuleGroup do not need to be in any specific ordering,
  // however, abusing of how it was built, specific types are expected.
  EXPECT_EQ(query_result.rule_group[0]->rule(0)->rule_type(),
            VehicleUsageRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->string(),
      VehicleUsageType::VehiclesAllowed()->string());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(0)->static_state().type()->value(),
      VehicleUsageType::VehiclesAllowed()->value());
  EXPECT_EQ(query_result.rule_group[0]->rule(1)->rule_type(),
            VehicleMotorizationRule::type());
  EXPECT_EQ(
      query_result.rule_group[0]->rule(1)->static_state().type()->string(),
      VehicleMotorizationType::MotorizedVehicles()->string());
  EXPECT_EQ(query_result.rule_group[0]->rule(1)->static_state().type()->value(),
            VehicleMotorizationType::MotorizedVehicles()->value());
}


}  // namespace
}  // namespace test
}  // namespace maliput
