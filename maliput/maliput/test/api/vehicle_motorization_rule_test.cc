/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/vehicle_motorization_rule.h"
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
using maliput::api::rules::VehicleMotorizationRule;
using maliput::api::rules::VehicleMotorizationState;
using maliput::api::rules::VehicleMotorizationType;


namespace maliput {
namespace api {
namespace rules {
namespace {

// VehicleMotorizationType construction test.
GTEST_TEST(VehicleMotorizationTypeTest, ConstructionTest) {
  auto motorized_vehicles_type = VehicleMotorizationType::MotorizedVehicles();
  auto non_motorized_vehicles_type =
      VehicleMotorizationType::NonMotorizedVehicles();
  auto unrestricted_vehicles_type =
      VehicleMotorizationType::UnrestrictedVehicles();

  EXPECT_NE(motorized_vehicles_type->value(),
            non_motorized_vehicles_type->value());
  EXPECT_NE(motorized_vehicles_type->value(),
            unrestricted_vehicles_type->value());
  EXPECT_NE(non_motorized_vehicles_type->value(),
            unrestricted_vehicles_type->value());

  EXPECT_EQ(motorized_vehicles_type->string(), "kMotorizedVehicles");
  EXPECT_EQ(non_motorized_vehicles_type->string(), "kNonMotorizedVehicles");
  EXPECT_EQ(unrestricted_vehicles_type->string(), "kUnrestrictedVehicles");
}

// VehicleMotorizationState construction test
GTEST_TEST(VehicleMotorizationStateTest, ConstructionTest) {
  const RuleState::Id kId("vms_id");
  const RuleState::Severity kSeverity{RuleState::Severity::kPreferred};
  auto vehicle_motorization_type = VehicleMotorizationType::MotorizedVehicles();

  const VehicleMotorizationState dut(
      kId, kSeverity, VehicleMotorizationType::MotorizedVehicles());

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.severity(), kSeverity);
  ASSERT_NE(dut.type(), nullptr);
  EXPECT_EQ(dut.type()->value(), vehicle_motorization_type->value());
  EXPECT_EQ(dut.type()->string(), vehicle_motorization_type->string());
}

// VehicleMotorizationRule construction test
GTEST_TEST(VehicleMotorizationRuleTest, ConstructionTest) {
  const RuleState::Id kStateId("vms_id");
  const RuleState::Severity kSeverity{RuleState::Severity::kPreferred};
  auto vehicle_motorization_type = VehicleMotorizationType::MotorizedVehicles();

  const RuleBase::Id kRuleId("r_id");
  const maliput::api::LaneId kLaneId("l_id");
  const SRange kSRange(0., 100.);
  const LaneSRange kZone(kLaneId, kSRange);
  std::vector<std::unique_ptr<RuleState>> states;
  states.push_back(std::make_unique<VehicleMotorizationState>(
      kStateId, kSeverity, VehicleMotorizationType::MotorizedVehicles()));

  const VehicleMotorizationRule dut(kRuleId, kZone, std::move(states));

  EXPECT_EQ(dut.id(), kRuleId);
  EXPECT_EQ(dut.zone().lane_id(), kZone.lane_id());
  EXPECT_EQ(dut.zone().s_range().s0(), kZone.s_range().s0());
  EXPECT_EQ(dut.zone().s_range().s1(), kZone.s_range().s1());
  EXPECT_TRUE(dut.is_static());
  EXPECT_EQ(dut.states().size(), 1);
  EXPECT_NE(dut.states().at(kStateId).get(), nullptr);
  EXPECT_EQ(dut.static_state().id(), kStateId);
  EXPECT_EQ(dut.rule_type(), VehicleMotorizationRule::type());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
