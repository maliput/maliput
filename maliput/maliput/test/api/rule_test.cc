/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/rule.h"
/* clang-format on */

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

#include "maliput/api/rules/regions.h"

using maliput::api::LaneId;
using maliput::api::rules::LaneSRange;
using maliput::api::rules::RuleBase;
using maliput::api::rules::RuleGroup;
using maliput::api::rules::RuleState;
using maliput::api::rules::RuleStateType;
using maliput::api::rules::SRange;

namespace maliput {
namespace api {
namespace rules {
namespace {

static const int kRuleStateTypeValue{123};
static const std::string kRuleStateTypeName{"MyRuleStateType"};

class MyRuleStateType : public RuleStateType {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyRuleStateType);

  static std::unique_ptr<MyRuleStateType> Mock() {
    return std::unique_ptr<MyRuleStateType>(new MyRuleStateType());
  }

 private:
  MyRuleStateType() : RuleStateType(kRuleStateTypeValue, kRuleStateTypeName) {}
};

// RuleStateType construction test.
GTEST_TEST(RuleStateTypeTest, ConstructionTest) {
  auto dut = MyRuleStateType::Mock();

  EXPECT_EQ(dut->value(), kRuleStateTypeValue);
  EXPECT_EQ(dut->string(), kRuleStateTypeName);
}

// RuleState construction test.
GTEST_TEST(RuleStateTest, ConstructionTest) {
  const RuleState::Id kId("rs_id");
  const RuleState::Severity kSeverity{RuleState::Severity::kPreferred};
  const RuleState dut(kId, kSeverity, MyRuleStateType::Mock());

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.severity(), kSeverity);
  ASSERT_NE(dut.type(), nullptr);
  EXPECT_EQ(dut.type()->value(), kRuleStateTypeValue);
  EXPECT_EQ(dut.type()->string(), kRuleStateTypeName);
}

// Rule construction test.
GTEST_TEST(RuleTest, ConstructionTest) {
  const RuleState::Id kStateId("rs_id");
  const RuleState::Severity kSeverity{RuleState::Severity::kPreferred};

  const RuleBase::Id kRuleId("r_id");
  const RuleBase::RuleTypeId kRuleTypeId("rt_id");
  const LaneSRange kZone(LaneId("l_id"), SRange(0., 100.));
  std::vector<std::unique_ptr<RuleState>> states;
  states.push_back(std::make_unique<RuleState>(
      kStateId, kSeverity, MyRuleStateType::Mock()));

  const RuleBase dut(kRuleId, kZone, kRuleTypeId, std::move(states));

  EXPECT_EQ(dut.id(), kRuleId);
  EXPECT_EQ(dut.zone().lane_id(), kZone.lane_id());
  EXPECT_EQ(dut.zone().s_range().s0(), kZone.s_range().s0());
  EXPECT_EQ(dut.zone().s_range().s1(), kZone.s_range().s1());
  EXPECT_TRUE(dut.is_static());
  EXPECT_EQ(dut.states().size(), 1);
  EXPECT_NE(dut.states().at(kStateId).get(), nullptr);
  EXPECT_EQ(dut.static_state().id(), kStateId);
  EXPECT_EQ(dut.rule_type(), kRuleTypeId);
}

// RuleGroup construction test.
GTEST_TEST(RuleGroupTest, ConstructionTest) {
  const RuleState::Id kStateId("rs_id");
  const RuleState::Severity kSeverity{RuleState::Severity::kPreferred};

  const RuleBase::Id kRuleId("r_id");
  const RuleBase::RuleTypeId kRuleTypeId("rt_id");
  const LaneSRange kZone(LaneId("l_id"), SRange(0., 100.));
  std::vector<std::unique_ptr<RuleState>> states;
  states.push_back(std::make_unique<RuleState>(
      kStateId, kSeverity, MyRuleStateType::Mock()));

  std::vector<std::unique_ptr<RuleBase>> rules;
  rules.push_back(std::make_unique<RuleBase>(
      kRuleId, kZone, kRuleTypeId, std::move(states)));

  const RuleGroup::Id kRuleGroupId("rg_id");
  const RuleGroup dut(kRuleGroupId, std::move(rules));

  EXPECT_EQ(dut.id(), kRuleGroupId);
  EXPECT_EQ(dut.size(), 1);
  EXPECT_EQ(dut.rule(0)->id(), kRuleId);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
