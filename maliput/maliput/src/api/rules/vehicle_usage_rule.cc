#include "maliput/api/rules/vehicle_usage_rule.h"

#include <utility>

namespace maliput {
namespace api {
namespace rules {

std::unique_ptr<VehicleUsageType> VehicleUsageType::VehiclesAllowed() {
  return std::unique_ptr<VehicleUsageType>(new VehicleUsageType(
      VehicleUsageType::Usage::kVehiclesAllowed, "kVehiclesAllowed"));
}

std::unique_ptr<VehicleUsageType> VehicleUsageType::VehiclesNotAllowed() {
  return std::unique_ptr<VehicleUsageType>(new VehicleUsageType(
      VehicleUsageType::Usage::kVehiclesNotAllowed, "kVehiclesNotAllowed"));
}

std::unordered_map<int, const char*, drake::DefaultHash>
VehicleUsageRule::StateTypeMapper() {
  auto vehicles_allowed = VehicleUsageType::VehiclesAllowed();
  auto vehicles_not_allowed = VehicleUsageType::VehiclesNotAllowed();

  return {
      {vehicles_allowed->value(), vehicles_allowed->string().c_str()},
      {vehicles_not_allowed->value(), vehicles_not_allowed->string().c_str()}};
}

RuleBase::RuleTypeId VehicleUsageRule::type() {
	return RuleBase::RuleTypeId("VehicleUsageRule");
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
