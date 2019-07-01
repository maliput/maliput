#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/type_specific_identifier.h"

namespace maliput {
namespace api {
namespace rules {

/// Vehicle motorization type describing whether the vehicle motor type can
/// travel along a Lane.
///
/// This class acts as an enumeration providing only certain instances through
/// its static methods.
class VehicleMotorizationType : public RuleStateType {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VehicleMotorizationType)
  VehicleMotorizationType() = delete;

  /// Motorized vehicles are allowed.
  static std::unique_ptr<VehicleMotorizationType> MotorizedVehicles();

  /// Non motorized vehicles are allowed.
  static std::unique_ptr<VehicleMotorizationType> NonMotorizedVehicles();

  /// Any vehicle type is allowed.
  static std::unique_ptr<VehicleMotorizationType> UnrestrictedVehicles();

 private:
  // Private enumeration to match literals to different RuleStateType values.
  enum MotorizationType {
    kMotorizedVehicles,
    kNonMotorizedVehicles,
    kUnrestrictedVehicles,
  };

  // Constructs a VehicleMotorizationType.
  VehicleMotorizationType(MotorizationType value, const std::string& type) :
      RuleStateType(static_cast<int>(value), type) {}
};

/// Semantic state of the VehicleMotorizationRule.
///
/// A RuleState that describes the current vehicle motorization semantics of the
/// lane section.
class VehicleMotorizationState : public RuleState {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VehicleMotorizationState)
  VehicleMotorizationState() = delete;

  /// Constructs a VehicleMotorizationState
  ///
  /// @param id the unique Id
  /// @param severity the Severity of the State
  /// @param type the semantic Type. It must not be nullptr and must be
  ///        VehicleMotorizationType convertible.
  VehicleMotorizationState(
      const RuleState::Id& id, RuleState::Severity severity,
      std::unique_ptr<RuleStateType> type)
        : RuleState(id, severity, std::move(type)) {
    DRAKE_DEMAND(
        dynamic_cast<const VehicleMotorizationType*>(type_.get()) != nullptr);
  }
};

/// Rule describing the allowed vehicle motorization type in a zone.
///
/// VehicleMotorizationRules are comprised of:
/// * a zone (a LaneSRange) which specifies the longitudinal section of the
/// road-network to which the rule instance applies.
/// * a catalog of one or more VehicleMotorizationState, each of which indicate
/// the possible VehicleMotorizationRule semantics for a vehicle traversing the
/// zone.
///
/// This rule classifies the motorization type of a vehicle. It is thought to be
/// part of a RuleGroup with a VehicleUsageRule.
///
/// Each Lane location can be governed by at most one VehicleMotorizationRule.
class VehicleMotorizationRule : public RuleBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VehicleMotorizationRule);

  /// Constructs a VehicleMotorizationRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param states a vector of valid states for the rule
  VehicleMotorizationRule(
      const RuleBase::Id& id, const LaneSRange& zone,
      std::vector<std::unique_ptr<RuleState>> states)
        : RuleBase(id, zone, type(), std::move(states)) {}

  /// Maps VehicleMotorizationType to string representations.
  static std::unordered_map<int, const char*, drake::DefaultHash>
  StateTypeMapper();

  /// Returns a RuleTypeId initialized with this class name.
  static RuleBase::RuleTypeId type();
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
