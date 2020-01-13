#include "maliput/base/rule_registry_loader.h"

#include <fstream>
#include <fmt/format.h>
#include "fmt/ostream.h"

#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule_registry.h"
#include "maliput/base/rule_registry.h"
#include "maliput/common/filesystem.h"

namespace maliput {
namespace {

// Evaluates the loader by creating a temporary file. The test will remove the temporary
// file when TestLoadingRuleTypesFromYaml::TearDown() is called.
class TestLoadingRuleTypesFromYaml : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("LoadRuleTypesFromYamlTest");
    ASSERT_TRUE(common::Filesystem::create_directory(directory_));

    rule_types_string_ = GenerateRuleTypesString();
    filepath_ = directory_.get_path() + "/rule_types_test.yaml";

    GenerateYamlFileFromString(rule_types_string_, filepath_);
  }
  void TearDown() override {
    if (!filepath_.empty()) {
      EXPECT_TRUE(common::Filesystem::remove_file(common::Path(filepath_)));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(directory_));
  }
  // Creates three rule types.
  std::string GenerateRuleTypesString() {
    const std::string kRightOfWayRuleType = RightOfWayRuleTypeId().string();
    const std::string kDirectionUsageRuleType = DirectionUsageRuleTypeId().string();
    const std::string kSpeedLimitRuleType = SpeedLimitRuleTypeId().string();
    const std::string kVehicleInStopBehaviour = VehicleStopInZoneBehaviorRuleTypeId().string();

    return fmt::format(
        R"R(RuleRegistry:
  {}:
    - value: "Go"
      severity: 0
      related_rules: [{}, {}]
      related_unique_ids: [{}]
    - value: "Stop"
      severity: 1
      related_rules: [{}]
    - value: "StopThenGo"
  {}:
    - value: "Bidirectional"
      severity: 0
    - value: "WithS"
      severity: 0
    - value: "AgaintsS"
      severity: 0
    - value: "Undefined"
      severity: 1
  {}:
    - range: [16.6, 27.8]
      description: "Interstate highway - day time"
      severity: 0
    - range: [16.6, 22.2]
      description: "Arterial road in Ciudad Autonoma de Buenos Aires"
      severity: 1
)R",
        kRightOfWayRuleType, kVehicleInStopBehaviour, RelatedRulesKeys::kYieldGroup, RelatedUniqueIdsKeys::kBulbGroup,
        kVehicleInStopBehaviour, kDirectionUsageRuleType, kSpeedLimitRuleType);
  }

  void GenerateYamlFileFromString(const std::string& string_to_yaml, const std::string& filepath) {
    std::ofstream os(filepath);
    fmt::print(os, string_to_yaml);
  }

  common::Path directory_;
  std::string rule_types_string_;
  std::string filepath_;
};

TEST_F(TestLoadingRuleTypesFromYaml, LoadFromFile) {
  const std::unique_ptr<api::rules::RuleRegistry> rule_registry = maliput::LoadRuleRegistryFromFile(filepath_);
  const int kStrict = api::rules::Rule::State::kStrict;
  const int kBestEffort = api::rules::Rule::State::kBestEffort;
  {
    const std::optional<api::rules::RuleRegistry::QueryResult> right_of_way_rule_states =
        rule_registry->GetPossibleStatesOfRuleType(RightOfWayRuleTypeId());
    EXPECT_TRUE(right_of_way_rule_states.has_value());
    EXPECT_EQ(right_of_way_rule_states->type_id, RightOfWayRuleTypeId());
    EXPECT_TRUE(right_of_way_rule_states->discrete_values.has_value());
    for (const auto& discrete_value : *(right_of_way_rule_states->discrete_values)) {
      if (discrete_value.value == "Go") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_NE(discrete_value.related_rules.find(VehicleStopInZoneBehaviorRuleTypeId().string()),
                  discrete_value.related_rules.end());
        EXPECT_NE(discrete_value.related_rules.find(RelatedRulesKeys::kYieldGroup), discrete_value.related_rules.end());
        EXPECT_NE(discrete_value.related_unique_ids.find(RelatedUniqueIdsKeys::kBulbGroup),
                  discrete_value.related_unique_ids.end());
      } else if (discrete_value.value == "Stop") {
        EXPECT_EQ(discrete_value.severity, kBestEffort);
        EXPECT_NE(discrete_value.related_rules.find(VehicleStopInZoneBehaviorRuleTypeId().string()),
                  discrete_value.related_rules.end());
        EXPECT_EQ(discrete_value.related_unique_ids.size(), 0);
      } else if (discrete_value.value == "StopThenGo") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_EQ(discrete_value.related_rules.size(), 0);
        EXPECT_EQ(discrete_value.related_unique_ids.size(), 0);
      } else {
        ADD_FAILURE() << "No more values for this rule type";
      }
    }
  }
  {
    const std::optional<api::rules::RuleRegistry::QueryResult> direction_usage_rule_states =
        rule_registry->GetPossibleStatesOfRuleType(DirectionUsageRuleTypeId());
    EXPECT_TRUE(direction_usage_rule_states.has_value());
    EXPECT_EQ(direction_usage_rule_states->type_id, DirectionUsageRuleTypeId());
    EXPECT_TRUE(direction_usage_rule_states->discrete_values.has_value());

    for (const auto& discrete_value : *(direction_usage_rule_states->discrete_values)) {
      if (discrete_value.value == "Bidirectional") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_EQ(discrete_value.related_rules.size(), 0);
        EXPECT_EQ(discrete_value.related_unique_ids.size(), 0);
      } else if (discrete_value.value == "WithS") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_EQ(discrete_value.related_rules.size(), 0);
        EXPECT_EQ(discrete_value.related_unique_ids.size(), 0);
      } else if (discrete_value.value == "AgaintsS") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_EQ(discrete_value.related_rules.size(), 0);
        EXPECT_EQ(discrete_value.related_unique_ids.size(), 0);
      } else if (discrete_value.value == "Undefined") {
        EXPECT_EQ(discrete_value.severity, kBestEffort);
        EXPECT_EQ(discrete_value.related_rules.size(), 0);
        EXPECT_EQ(discrete_value.related_unique_ids.size(), 0);
      } else {
        ADD_FAILURE() << "No more values for this rule type";
      }
    }
  }
  {
    const std::optional<api::rules::RuleRegistry::QueryResult> speed_limit_rule_states =
        rule_registry->GetPossibleStatesOfRuleType(SpeedLimitRuleTypeId());
    EXPECT_TRUE(speed_limit_rule_states.has_value());
    EXPECT_EQ(speed_limit_rule_states->type_id, SpeedLimitRuleTypeId());
    EXPECT_TRUE(speed_limit_rule_states->range_values.has_value());

    for (const auto& range_value : *(speed_limit_rule_states->range_values)) {
      if (range_value.description == "Interstate highway - day time") {
        EXPECT_DOUBLE_EQ(range_value.min, 16.6);
        EXPECT_DOUBLE_EQ(range_value.max, 27.8);
        EXPECT_EQ(range_value.severity, kStrict);
        EXPECT_EQ(range_value.related_rules.size(), 0);
        EXPECT_EQ(range_value.related_unique_ids.size(), 0);
      } else if (range_value.description == "Arterial road in Ciudad Autonoma de Buenos Aires") {
        EXPECT_DOUBLE_EQ(range_value.min, 16.6);
        EXPECT_DOUBLE_EQ(range_value.max, 22.2);
        EXPECT_EQ(range_value.severity, kBestEffort);
        EXPECT_EQ(range_value.related_rules.size(), 0);
        EXPECT_EQ(range_value.related_unique_ids.size(), 0);
      } else {
        ADD_FAILURE() << "No more values for this rule type";
      }
    }
  }
}

}  // namespace

}  // namespace maliput
