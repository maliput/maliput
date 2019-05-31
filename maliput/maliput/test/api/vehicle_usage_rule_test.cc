/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/vehicle_usage_rule.h"
/* clang-format on */

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>


#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/rule.h"
#include "drake/common/drake_throw.h"

using maliput::api::LaneId;
using maliput::api::rules::LaneSRange;
using maliput::api::rules::RuleState;
using maliput::api::rules::SRange;
using maliput::api::rules::VehicleUsageRule;
using maliput::api::rules::VehicleUsageState;
using maliput::api::rules::VehicleUsageType;


namespace maliput {
namespace api {
namespace rules {
namespace {

// VehicleUsageType construction test.
GTEST_TEST(VehicleUsageTypeTest, ConstructionTest) {
  auto vehicles_allowed_type = VehicleUsageType::VehiclesAllowed();
  auto vehicles_not_allowed_type = VehicleUsageType::VehiclesNotAllowed();

  EXPECT_NE(vehicles_allowed_type->value(), vehicles_not_allowed_type->value());
  EXPECT_EQ(vehicles_allowed_type->string(), "kVehiclesAllowed");
  EXPECT_EQ(vehicles_not_allowed_type->string(), "kVehiclesNotAllowed");
}

// VehicleUsageState construction test.
GTEST_TEST(VehicleUsageStateTest, ConstructionTest) {
  const RuleState::Id kId("vus_id");
  const RuleState::Severity kSeverity{RuleState::Severity::kPreferred};
  auto vehicle_usage_type = VehicleUsageType::VehiclesAllowed();

  const VehicleUsageState dut(
      kId, kSeverity, VehicleUsageType::VehiclesAllowed());

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.severity(), kSeverity);
  ASSERT_NE(dut.type(), nullptr);
  EXPECT_EQ(dut.type()->value(), vehicle_usage_type->value());
  EXPECT_EQ(dut.type()->string(), vehicle_usage_type->string());
}

// VehicleUsageRule construction test.
GTEST_TEST(VehicleUsageRuleTest, ConstructionTest) {
  const RuleState::Id kStateId("vus_id");
  const RuleState::Severity kSeverity{RuleState::Severity::kPreferred};
  auto vehicle_usage_type = VehicleUsageType::VehiclesAllowed();

  const RuleBase::Id kRuleId("r_id");
  const maliput::api::LaneId kLaneId("l_id");
  const SRange kSRange(0., 100.);
  const LaneSRange kZone(kLaneId, kSRange);
  std::vector<std::unique_ptr<RuleState>> states;
  states.push_back(std::make_unique<VehicleUsageState>(
      kStateId, kSeverity, VehicleUsageType::VehiclesAllowed()));

  const VehicleUsageRule dut(kRuleId, kZone, std::move(states));

  EXPECT_EQ(dut.id(), kRuleId);
  EXPECT_EQ(dut.zone().lane_id(), kZone.lane_id());
  EXPECT_EQ(dut.zone().s_range().s0(), kZone.s_range().s0());
  EXPECT_EQ(dut.zone().s_range().s1(), kZone.s_range().s1());
  EXPECT_TRUE(dut.is_static());
  EXPECT_EQ(dut.states().size(), 1);
  EXPECT_NE(dut.states().at(kStateId).get(), nullptr);
  EXPECT_EQ(dut.static_state().id(), kStateId);
  EXPECT_EQ(dut.rule_type(), VehicleUsageRule::type());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
