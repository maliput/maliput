#include "maliput/base/road_rulebook_loader.h"

#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <fstream>
#include <fmt/format.h>
#include "fmt/ostream.h"

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/api/unique_id.h"
#include "maliput/base/rule_registry.h"
#include "maliput/base/traffic_light_book_loader.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_direction_usage_compare.h"
#include "maliput/test_utilities/rules_right_of_way_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

#include "drake/common/find_resource.h"

namespace maliput {
namespace {

constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::SRange;
using maliput::api::UniqueId;
using maliput::api::rules::BulbGroup;
using maliput::api::rules::DirectionUsageRule;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::Rule;
using maliput::api::rules::TrafficLight;
using maliput::api::rules::UniqueBulbGroupId;

class TestLoading2x2IntersectionRules : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionRules()
      : filepath_(maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) + "/2x2_intersection.yaml"),
        road_geometry_(multilane::LoadFile(multilane::BuilderFactory(), filepath_)) {}

  std::vector<RightOfWayRule> CreateStraightThroughRightOfWayRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
      std::string traffic_light_name;
      std::string bulb_group_name;
    };
    const std::vector<TestCase> test_cases = {
        {"NorthStraight", "l:ns_intersection_segment_0", "SouthFacing", "SouthFacingBulbs"},
        {"SouthStraight", "l:ns_intersection_segment_1", "NorthFacing", "NorthFacingBulbs"},
        {"EastStraight", "l:ew_intersection_segment_0", "WestFacing", "WestFacingBulbs"},
        {"WestStraight", "l:ew_intersection_segment_1", "EastFacing", "EastFacingBulbs"},
    };
    std::vector<RightOfWayRule> result;
    for (const auto& test_case : test_cases) {
      result.push_back(RightOfWayRule(
          RightOfWayRule::Id(test_case.rule_name),
          LaneSRoute({LaneSRange(LaneId(test_case.lane_name), SRange(0, 18.75))}),
          RightOfWayRule::ZoneType::kStopExcluded,
          {RightOfWayRule::State(RightOfWayRule::State::Id("Go"), RightOfWayRule::State::Type::kGo, {}),
           RightOfWayRule::State(RightOfWayRule::State::Id("Stop"), RightOfWayRule::State::Type::kStop, {})},
          RightOfWayRule::RelatedBulbGroups{
              {TrafficLight::Id(test_case.traffic_light_name), {BulbGroup::Id(test_case.bulb_group_name)}}}));
    }
    return result;
  }

  std::vector<RightOfWayRule> CreateRightTurnRightOfWayRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
      std::string yield_1;
      std::string yield_2;
      std::string traffic_light_name;
      std::string bulb_group_name;
    };
    const std::vector<TestCase> test_cases = {
        {"NorthRightTurn", "l:north_right_turn_segment_0", "EastStraight", "SouthLeftTurn", "SouthFacing",
         "SouthFacingBulbs"},
        {"SouthRightTurn", "l:south_right_turn_segment_0", "WestStraight", "NorthLeftTurn", "NorthFacing",
         "NorthFacingBulbs"},
        {"EastRightTurn", "l:east_right_turn_segment_0", "SouthStraight", "WestLeftTurn", "WestFacing",
         "WestFacingBulbs"},
        {"WestRightTurn", "l:west_right_turn_segment_0", "NorthStraight", "EastLeftTurn", "EastFacing",
         "EastFacingBulbs"},
    };
    std::vector<RightOfWayRule> result;
    for (const auto& test_case : test_cases) {
      result.push_back(RightOfWayRule(
          RightOfWayRule::Id(test_case.rule_name),
          LaneSRoute({LaneSRange(LaneId(test_case.lane_name), SRange(0, M_PI_2 * 7.5))}),
          RightOfWayRule::ZoneType::kStopExcluded,
          {RightOfWayRule::State(RightOfWayRule::State::Id("Go"), RightOfWayRule::State::Type::kGo,
                                 {RightOfWayRule::Id(test_case.yield_2)}),
           RightOfWayRule::State(RightOfWayRule::State::Id("StopThenGo"), RightOfWayRule::State::Type::kStopThenGo,
                                 {RightOfWayRule::Id(test_case.yield_1), RightOfWayRule::Id(test_case.yield_2)})},
          RightOfWayRule::RelatedBulbGroups{
              {TrafficLight::Id(test_case.traffic_light_name), {BulbGroup::Id(test_case.bulb_group_name)}}}));
    }
    return result;
  }

  std::vector<RightOfWayRule> CreateLeftTurnRightOfWayRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
      std::string yield;
      std::string traffic_light_name;
      std::string bulb_group_name;
    };
    std::vector<TestCase> test_cases = {
        {"NorthLeftTurn", "l:north_left_turn_segment_0", "SouthStraight", "SouthFacing", "SouthFacingBulbs"},
        {"SouthLeftTurn", "l:south_left_turn_segment_0", "NorthStraight", "NorthFacing", "NorthFacingBulbs"},
        {"EastLeftTurn", "l:east_left_turn_segment_0", "WestStraight", "WestFacing", "WestFacingBulbs"},
        {"WestLeftTurn", "l:west_left_turn_segment_0", "EastStraight", "EastFacing", "EastFacingBulbs"},
    };
    std::vector<RightOfWayRule> result;
    for (const auto& test_case : test_cases) {
      result.push_back(RightOfWayRule(
          RightOfWayRule::Id(test_case.rule_name),
          LaneSRoute({LaneSRange(LaneId(test_case.lane_name), SRange(0, M_PI_2 * 11.25))}),
          RightOfWayRule::ZoneType::kStopExcluded,
          {RightOfWayRule::State(RightOfWayRule::State::Id("Go"), RightOfWayRule::State::Type::kGo,
                                 {RightOfWayRule::Id(test_case.yield)}),
           RightOfWayRule::State(RightOfWayRule::State::Id("Stop"), RightOfWayRule::State::Type::kStop, {})},
          RightOfWayRule::RelatedBulbGroups{
              {TrafficLight::Id(test_case.traffic_light_name), {BulbGroup::Id(test_case.bulb_group_name)}}}));
    }
    return result;
  }

  std::vector<DirectionUsageRule> CreateDirectionUsageRules() const {
    struct TestCase {
      std::string rule_name;
      std::string lane_name;
      DirectionUsageRule::State::Type rule_type;
      DirectionUsageRule::State::Severity rule_severity;
    };
    const std::vector<TestCase> test_cases = {
        // Turn cases.
        {"NorthRightTurn", "l:north_right_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"NorthLeftTurn", "l:north_left_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"SouthRightTurn", "l:south_right_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"SouthLeftTurn", "l:south_left_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"EastRightTurn", "l:east_right_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"EastLeftTurn", "l:east_left_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"WestRightTurn", "l:west_right_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"WestLeftTurn", "l:west_left_turn_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        // Straight segments.
        {"NorthApproach", "l:s_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"NorthStraight", "l:ns_intersection_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"NorthExit", "l:n_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"SouthApproach", "l:n_segment_1", DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"SouthStraight", "l:ns_intersection_segment_1", DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kStrict},
        {"SouthExit", "l:s_segment_1", DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"EastApproach", "l:w_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"EastStraight", "l:ew_intersection_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kStrict},
        {"EastExit", "l:e_segment_0", DirectionUsageRule::State::Type::kWithS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"WestEntrance", "l:e_segment_1", DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
        {"WestStraight", "l:ew_intersection_segment_1", DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kStrict},
        {"WestExit", "l:w_segment_1", DirectionUsageRule::State::Type::kAgainstS,
         DirectionUsageRule::State::Severity::kPreferred},
    };
    std::vector<DirectionUsageRule> result;
    for (const auto& test_case : test_cases) {
      const auto lane = road_geometry_->ById().GetLane(LaneId(test_case.lane_name));
      result.push_back(DirectionUsageRule(DirectionUsageRule::Id(test_case.rule_name),
                                          LaneSRange(LaneId(test_case.lane_name), SRange(0, lane->length())),
                                          {DirectionUsageRule::State(DirectionUsageRule::State::Id("default"),
                                                                     test_case.rule_type, test_case.rule_severity)}));
    }
    return result;
  }

  // Evaluates that RelatedBulbGroups in `rule` has valid TrafficLight::Ids and
  // BulbGroup::Ids.
  void EvaluateRelatedBulbGroups(const RightOfWayRule& rule, const api::rules::TrafficLightBook* traffic_light_book) {
    for (const auto& traffic_light_bulb_groups : rule.related_bulb_groups()) {
      const api::rules::TrafficLight* traffic_light =
          traffic_light_book->GetTrafficLight(traffic_light_bulb_groups.first);
      EXPECT_NE(traffic_light, nullptr);
      for (const api::rules::BulbGroup::Id& bulb_group_id : traffic_light_bulb_groups.second) {
        EXPECT_NE(traffic_light->GetBulbGroup(bulb_group_id), nullptr);
      }
    }
  }

  Rule::Id GetRuleIdFrom(const Rule::TypeId& rule_type_id, const RightOfWayRule::Id& right_of_way_rule_id) {
    return maliput::api::rules::Rule::Id(rule_type_id.string() + "/" + right_of_way_rule_id.string());
  }

  // Verifies that for each RightOfWayRule two DiscreteValueRules of types Right-Of-Way and
  // Vehicle Stop In Zone Behavior Rule Types are created.
  void VerifyDiscreteValueRuleEquivalents(const api::rules::RoadRulebook* rulebook,
                                          const RightOfWayRule& right_of_way_rule) {
    const std::unordered_map<RightOfWayRule::State::Type, std::string> right_of_way_rule_state_types{
        {RightOfWayRule::State::Type::kGo, "Go"},
        {RightOfWayRule::State::Type::kStop, "Stop"},
        {RightOfWayRule::State::Type::kStopThenGo, "StopThenGo"},
    };

    EXPECT_NE(rulebook, nullptr);
    // Check that discrete value rule with a Vehicle Stop In Zone Behaviour Rule Type was created.
    const DiscreteValueRule vehicle_stop_in_zone_discrete_rule =
        rulebook->GetDiscreteValueRule(GetRuleIdFrom(VehicleStopInZoneBehaviorRuleTypeId(), right_of_way_rule.id()));
    // Compare values.
    EXPECT_DOUBLE_EQ(vehicle_stop_in_zone_discrete_rule.zone().length(), right_of_way_rule.zone().length());
    EXPECT_EQ(vehicle_stop_in_zone_discrete_rule.values().size(), 1);
    EXPECT_EQ(vehicle_stop_in_zone_discrete_rule.values()[0].severity, {Rule::State::kStrict});
    EXPECT_TRUE(vehicle_stop_in_zone_discrete_rule.values()[0].related_rules.empty());
    if (right_of_way_rule.zone_type() == RightOfWayRule::ZoneType::kStopExcluded) {
      EXPECT_EQ(vehicle_stop_in_zone_discrete_rule.values()[0].value, "DoNotStop");
    } else {
      EXPECT_EQ(vehicle_stop_in_zone_discrete_rule.values()[0].value, "UnconstrainedParking");
    }

    // Check that discrete value rule from the right of way rule type was created.
    const DiscreteValueRule right_of_way_discrete_rule =
        rulebook->GetDiscreteValueRule(GetRuleIdFrom(RightOfWayRuleTypeId(), right_of_way_rule.id()));

    EXPECT_DOUBLE_EQ(right_of_way_discrete_rule.zone().length(), right_of_way_rule.zone().length());
    // Check the matching between right of way rules and discrete value rules.
    for (const auto& state : right_of_way_rule.states()) {
      // Check if the state of the RightOfWayRule has correspondence with the value of the DiscreteValueRule.
      const auto discrete_value_it =
          find_if(right_of_way_discrete_rule.values().begin(), right_of_way_discrete_rule.values().end(),
                  [right_of_way_rule_state_types, state](DiscreteValueRule::DiscreteValue discrete_value) {
                    return discrete_value.value == right_of_way_rule_state_types.at(state.second.type());
                  });
      EXPECT_NE(discrete_value_it, right_of_way_discrete_rule.values().end());
      // Check the related rules of the discrete value.
      EXPECT_EQ(discrete_value_it->related_rules.at(VehicleStopInZoneBehaviorRuleTypeId().string())[0],
                GetRuleIdFrom(VehicleStopInZoneBehaviorRuleTypeId(), right_of_way_rule.id()));
      for (const auto yield_id : state.second.yield_to()) {
        auto it = std::find(discrete_value_it->related_rules.at(RelatedRulesKeys::kYieldGroup).begin(),
                            discrete_value_it->related_rules.at(RelatedRulesKeys::kYieldGroup).end(),
                            GetRuleIdFrom(RightOfWayRuleTypeId(), yield_id));
        EXPECT_NE(it, discrete_value_it->related_rules.at(RelatedRulesKeys::kYieldGroup).end());
      }
    }

    // Check the related unique ids of the discrete value.
    int related_bulb_group_size{0};
    for (const auto& traffic_light_id_vector_bulb_group_id : right_of_way_rule.related_bulb_groups()) {
      related_bulb_group_size += traffic_light_id_vector_bulb_group_id.second.size();
    }
    for (const auto& discrete_value : right_of_way_discrete_rule.values()) {
      EXPECT_EQ(discrete_value.related_unique_ids.at(RelatedUniqueIdsKeys::kBulbGroup).size(), related_bulb_group_size);
      for (const auto& unique_id : discrete_value.related_unique_ids.at(RelatedUniqueIdsKeys::kBulbGroup)) {
        const auto related_bulb_group_it = find_if(
            right_of_way_rule.related_bulb_groups().begin(), right_of_way_rule.related_bulb_groups().end(),
            [unique_id](std::pair<TrafficLight::Id, std::vector<BulbGroup::Id>> traffic_light_id_vector_bulb_group_id) {
              for (const auto& bulb_group_id : traffic_light_id_vector_bulb_group_id.second) {
                return unique_id == UniqueBulbGroupId{traffic_light_id_vector_bulb_group_id.first, bulb_group_id};
              }
              return false;
            });
        EXPECT_NE(related_bulb_group_it, right_of_way_rule.related_bulb_groups().end());
      }
    }
  }

  const std::string filepath_;
  const std::unique_ptr<const api::RoadGeometry> road_geometry_;
};

TEST_F(TestLoading2x2IntersectionRules, LoadFromFile) {
  const std::unique_ptr<api::rules::RoadRulebook> rulebook = LoadRoadRulebookFromFile(road_geometry_.get(), filepath_);
  EXPECT_NE(rulebook, nullptr);

  const std::unique_ptr<api::rules::TrafficLightBook> traffic_light_book = LoadTrafficLightBookFromFile(filepath_);

  // RightOfWayRules testing.
  {
    const auto straight_cases = CreateStraightThroughRightOfWayRules();
    const auto right_turn_cases = CreateRightTurnRightOfWayRules();
    const auto left_turn_cases = CreateLeftTurnRightOfWayRules();

    std::vector<RightOfWayRule> test_cases;
    test_cases.insert(test_cases.end(), std::begin(straight_cases), std::end(straight_cases));
    test_cases.insert(test_cases.end(), std::begin(right_turn_cases), std::end(right_turn_cases));
    test_cases.insert(test_cases.end(), std::begin(left_turn_cases), std::end(left_turn_cases));

    for (const auto& test_case : test_cases) {
      const RightOfWayRule rule = rulebook->GetRule(test_case.id());
      EXPECT_TRUE(MALIPUT_IS_EQUAL(rule, test_case));
      VerifyDiscreteValueRuleEquivalents(rulebook.get(), rule);
      EvaluateRelatedBulbGroups(rule, traffic_light_book.get());
    }
  }

  // DirectionUsageRules testing.
  {
    std::vector<DirectionUsageRule> direction_usage_cases = CreateDirectionUsageRules();
    for (const auto& test_case : direction_usage_cases) {
      const DirectionUsageRule rule = rulebook->GetRule(test_case.id());
      EXPECT_TRUE(MALIPUT_IS_EQUAL(rule, test_case));
    }
  }
}

// Evaluates the loader by creating a temporary file. The test will remove the temporary
// file when TestLoadingRulesFromYaml::TearDown() is called.
class TestLoadingRulesFromYaml : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("LoadRulebookFromYamlTest");
    ASSERT_TRUE(common::Filesystem::create_directory(directory_));

    rules_string_ = GenerateRulebookString();
    filepath_ = directory_.get_path() + "/rules_loader_test.yaml";

    GenerateYamlFileFromString(rules_string_, filepath_);
    road_geometry_ = api::test::CreateTwoLanesRoadGeometry();
    rule_registry_ = api::test::CreateBasicRuleRegistry();
  }
  void TearDown() override {
    if (!filepath_.empty()) {
      EXPECT_TRUE(common::Filesystem::remove_file(common::Path(filepath_)));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(directory_));
  }
  // Returns a YAML string representation of an api::rules::Rulebook with three rules with different rule types.
  std::string GenerateRulebookString() {
    return fmt::format(
        R"R(RoadRulebook:
  - id: "rule1"
    type: "Right-Of-Way Rule Type"
    zone:
      - lane_id: "lane_a"
        s_range: [0, 100]
      - lane_id: "lane_b"
        s_range: [0, 50]
    values:
      - value: "Go"
        severity: 0
        related_rules:
        - Yield Group: ["RightOfWayRuleType/lane2"]
          Vehicle Stop In Zone Behavior: ["VehicleStopInZoneBehaviorRuleType/lane34"]
        related_unique_ids:
        - Bulb Group: ["TrafficLightId-BulbGroupId"]
      - value: "Stop"
        severity: 0
        related_rules:
        - Yield Group: ["RightOfWayRuleType/lane2"]
        related_unique_ids:
        - Bulb Group: ["TrafficLightId-BulbGroupId"]
  - id: "rule2"
    type: "Speed-Limit Rule Type"
    zone:
      - lane_id: "lane_a"
        s_range: [0, 100]
      - lane_id: "lane_b"
        s_range: [0, 50]
    ranges:
      - range: [16.6, 27.8]
        description : "Interstate highway - day time"
        severity: 0
        related_rules: []
        related_unique_ids: []
)R");
  }

  // Creates a vector of api::rules::DiscreteValueRule.
  std::vector<api::rules::DiscreteValueRule> CreateDiscreteValueRules() {
    std::vector<DiscreteValueRule::DiscreteValue> discrete_values;
    discrete_values.push_back(
        {Rule::State::kStrict,
         Rule::RelatedRules{{"Yield Group", {Rule::Id{"RightOfWayRuleType/lane2"}}},
                            {"Vehicle Stop In Zone Behavior", {Rule::Id{"VehicleStopInZoneBehaviorRuleType/lane34"}}}},
         Rule::RelatedUniqueIds{{"Bulb Group", {UniqueId{"TrafficLightId-BulbGroupId"}}}}, "Go"});
    discrete_values.push_back(
        {Rule::State::kStrict, Rule::RelatedRules{{"Yield Group", {Rule::Id{"RightOfWayRuleType/lane2"}}}},
         Rule::RelatedUniqueIds{{"Bulb Group", {UniqueId{"TrafficLightId-BulbGroupId"}}}}, "Stop"});
    DiscreteValueRule discrete_value_rule{
        Rule::Id{"rule1"}, Rule::TypeId{"Right-Of-Way Rule Type"},
        LaneSRoute{{{LaneId{"lane_a"}, SRange{0., 100.}}, {LaneId{"lane_b"}, SRange{0., 50.}}}}, discrete_values};
    return {{discrete_value_rule}};
  }

  // Creates a vector of api::rules::RangeValueRules.
  std::vector<api::rules::RangeValueRule> CreateRangeValueRules() {
    std::vector<RangeValueRule::Range> range_values;
    range_values.push_back({Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                            "Interstate highway - day time", 16.6, 27.8});
    RangeValueRule range_value_rule{
        Rule::Id{"rule2"}, Rule::TypeId{"Speed-Limit Rule Type"},
        LaneSRoute{{{LaneId{"lane_a"}, SRange{0., 100.}}, {LaneId{"lane_b"}, SRange{0., 50.}}}}, range_values};
    return {{range_value_rule}};
  }

  // Generates a YAML file located in 'filepath' from 'string_to_yaml''s content.
  void GenerateYamlFileFromString(const std::string& string_to_yaml, const std::string& filepath) {
    std::ofstream os(filepath);
    fmt::print(os, string_to_yaml);
  }

  common::Path directory_;
  std::string rules_string_;
  std::string filepath_;
  std::unique_ptr<api::RoadGeometry> road_geometry_;
  std::unique_ptr<api::rules::RuleRegistry> rule_registry_;
};

TEST_F(TestLoadingRulesFromYaml, LoadFromFile) {
  std::unique_ptr<api::rules::RoadRulebook> rulebook =
      LoadRoadRulebookFromFile(road_geometry_.get(), filepath_, *rule_registry_.get());
  api::rules::RoadRulebook::QueryResults rules = rulebook->Rules();
  {  // Verify discrete value rules.
    const std::vector<api::rules::DiscreteValueRule> expected_discrete_rules = CreateDiscreteValueRules();
    const std::map<DiscreteValueRule::Id, DiscreteValueRule> ids_discrete_value_rules = rules.discrete_value_rules;
    EXPECT_EQ(ids_discrete_value_rules.size(), expected_discrete_rules.size());
    for (const auto& id_discrete_value_rule : ids_discrete_value_rules) {
      const auto it =
          std::find_if(expected_discrete_rules.begin(), expected_discrete_rules.end(),
                       [id_discrete_value_rule](api::rules::DiscreteValueRule expected_discrete_rule) {
                         return api::rules::test::IsEqual("DiscreteValueRule", "Expected DiscreteValueRule",
                                                          id_discrete_value_rule.second,
                                                          expected_discrete_rule) == testing::AssertionSuccess();
                         return api::rules::test::IsEqual(
                                    "DiscreteValues", "Expected DiscreteValues", id_discrete_value_rule.second.values(),
                                    expected_discrete_rule.values()) == testing::AssertionSuccess();
                       });
      EXPECT_NE(it, expected_discrete_rules.end());
    }
  }
  {  // Verify range value rules.
    const std::vector<api::rules::RangeValueRule> expected_range_rules = CreateRangeValueRules();
    const std::map<RangeValueRule::Id, RangeValueRule> ids_range_value_rules = rules.range_value_rules;
    EXPECT_EQ(rules.range_value_rules.size(), expected_range_rules.size());
    for (const auto& id_range_value_rule : ids_range_value_rules) {
      const auto it = std::find_if(
          expected_range_rules.begin(), expected_range_rules.end(),
          [id_range_value_rule](api::rules::RangeValueRule expected_range_rule) {
            return api::rules::test::IsEqual("RangeValueRule", "Expected RangeValueRule", id_range_value_rule.second,
                                             expected_range_rule) == testing::AssertionSuccess();
            return api::rules::test::IsEqual("RangeValues", "Expected RangeValues", id_range_value_rule.second.ranges(),
                                             expected_range_rule.ranges()) == testing::AssertionSuccess();
          });
      EXPECT_NE(it, expected_range_rules.end());
    }
  }
}

}  // namespace
}  // namespace maliput
