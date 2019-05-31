#include "maliput/api/rules/vehicle_motorization_rule.h"

namespace maliput {
namespace api {
namespace rules {

std::unique_ptr<VehicleMotorizationType>
VehicleMotorizationType::MotorizedVehicles() {
  return std::unique_ptr<VehicleMotorizationType>(new VehicleMotorizationType(
      VehicleMotorizationType::MotorizationType::kMotorizedVehicles,
      "kMotorizedVehicles"));
}

std::unique_ptr<VehicleMotorizationType>
VehicleMotorizationType::NonMotorizedVehicles() {
  return std::unique_ptr<VehicleMotorizationType>(new VehicleMotorizationType(
      VehicleMotorizationType::MotorizationType::kNonMotorizedVehicles,
      "kNonMotorizedVehicles"));
}

std::unique_ptr<VehicleMotorizationType>
VehicleMotorizationType::UnrestrictedVehicles() {
  return std::unique_ptr<VehicleMotorizationType>(new VehicleMotorizationType(
      VehicleMotorizationType::MotorizationType::kUnrestrictedVehicles,
      "kUnrestrictedVehicles"));
}

std::unordered_map<int, const char*, drake::DefaultHash>
VehicleMotorizationRule::StateTypeMapper() {
  auto motorized_vehicles = VehicleMotorizationType::MotorizedVehicles();
  auto nonmotorized_vehicles = VehicleMotorizationType::NonMotorizedVehicles();
  auto unrestricted_vehicles = VehicleMotorizationType::UnrestrictedVehicles();

  return {
      {motorized_vehicles->value(), motorized_vehicles->string().c_str()},
      {nonmotorized_vehicles->value(), nonmotorized_vehicles->string().c_str()},
      {unrestricted_vehicles->value(),
        unrestricted_vehicles->string().c_str()}};
}

RuleBase::RuleTypeId VehicleMotorizationRule::type() {
  return RuleBase::RuleTypeId("VehicleMotorizationRule");
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
