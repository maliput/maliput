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
#include "maliput/base/rule_registry_loader.h"

#include <fstream>
#include <variant>

#include <fmt/format.h>
#include <fmt/ostream.h>
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
    const auto discrete_values_ptr =
        std::get_if<api::rules::RuleRegistry::QueryResult::DiscreteValues>(&right_of_way_rule_states->rule_values);
    EXPECT_NE(discrete_values_ptr, nullptr);

    for (const auto& discrete_value : *discrete_values_ptr) {
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
        EXPECT_TRUE(discrete_value.related_unique_ids.empty());
      } else if (discrete_value.value == "StopThenGo") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_TRUE(discrete_value.related_rules.empty());
        EXPECT_TRUE(discrete_value.related_unique_ids.empty());
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
    const auto discrete_values_ptr =
        std::get_if<api::rules::RuleRegistry::QueryResult::DiscreteValues>(&direction_usage_rule_states->rule_values);
    EXPECT_NE(discrete_values_ptr, nullptr);

    for (const auto& discrete_value : *discrete_values_ptr) {
      if (discrete_value.value == "Bidirectional") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_TRUE(discrete_value.related_rules.empty());
        EXPECT_TRUE(discrete_value.related_unique_ids.empty());
      } else if (discrete_value.value == "WithS") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_TRUE(discrete_value.related_rules.empty());
        EXPECT_TRUE(discrete_value.related_unique_ids.empty());
      } else if (discrete_value.value == "AgaintsS") {
        EXPECT_EQ(discrete_value.severity, kStrict);
        EXPECT_TRUE(discrete_value.related_rules.empty());
        EXPECT_TRUE(discrete_value.related_unique_ids.empty());
      } else if (discrete_value.value == "Undefined") {
        EXPECT_EQ(discrete_value.severity, kBestEffort);
        EXPECT_TRUE(discrete_value.related_rules.empty());
        EXPECT_TRUE(discrete_value.related_unique_ids.empty());
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
    const auto range_values_ptr =
        std::get_if<api::rules::RuleRegistry::QueryResult::Ranges>(&speed_limit_rule_states->rule_values);
    EXPECT_NE(range_values_ptr, nullptr);

    for (const auto& range_value : *range_values_ptr) {
      if (range_value.description == "Interstate highway - day time") {
        EXPECT_DOUBLE_EQ(range_value.min, 16.6);
        EXPECT_DOUBLE_EQ(range_value.max, 27.8);
        EXPECT_EQ(range_value.severity, kStrict);
        EXPECT_TRUE(range_value.related_rules.empty());
        EXPECT_TRUE(range_value.related_unique_ids.empty());
      } else if (range_value.description == "Arterial road in Ciudad Autonoma de Buenos Aires") {
        EXPECT_DOUBLE_EQ(range_value.min, 16.6);
        EXPECT_DOUBLE_EQ(range_value.max, 22.2);
        EXPECT_EQ(range_value.severity, kBestEffort);
        EXPECT_TRUE(range_value.related_rules.empty());
        EXPECT_TRUE(range_value.related_unique_ids.empty());
      } else {
        ADD_FAILURE() << "No more values for this rule type";
      }
    }
  }
}

}  // namespace

}  // namespace maliput
