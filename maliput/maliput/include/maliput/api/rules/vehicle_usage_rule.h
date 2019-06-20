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

/// Vehicle usage type describing whether or not vehicles can travel along a
/// Lane.
///
/// This class acts as an enumeration providing only certain instances through
/// its static methods.
class VehicleUsageType : public RuleStateType {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VehicleUsageType)
  VehicleUsageType() = delete;

  /// Vehicles are allowed to travel along a Lane zone.
  static std::unique_ptr<VehicleUsageType> VehiclesAllowed();

  /// Vehicles are not allowed to travel along a Lane zone.
  static std::unique_ptr<VehicleUsageType> VehiclesNotAllowed();

 private:
  // Private enumeration to match literals to different RuleStateType values.
  enum class Usage {
    kVehiclesAllowed,
    kVehiclesNotAllowed
  };

  // Constructs a VehicleUsageType
  VehicleUsageType(Usage value, const std::string& type) :
      RuleStateType(static_cast<int>(value), type) {}
};

/// Semantic state of the VehicleUsageRule.
///
/// A RuleState that describes the current vehicle usage semantics of the lane
/// section.
class VehicleUsageState : public RuleState {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VehicleUsageState);

  /// Constructs a VehicleUsageState
  ///
  /// @param id the unique Id
  /// @param severity the Severity of the State
  /// @param type the semantic Type. It must not be nullptr and must be
  ///        VehicleUsageType convertible.
  VehicleUsageState(const RuleState::Id& id, RuleState::Severity severity,
                    std::unique_ptr<RuleStateType> type)
      : RuleState(id, severity, std::move(type)) {
    DRAKE_DEMAND(dynamic_cast<const VehicleUsageType*>(type_.get()) != nullptr);
  }
};

/// Rule describing whether vehicles are allowed or not in a zone.
///
/// VehicleUsageRules are comprised of:
/// * a zone (a LaneSRange) which specifies the longitudinal section of the
/// road-network to which the rule instance applies.
/// * a catalog of one or more VehicleUsageStates, each of which indicate the
/// possible VehicleUsageRule semantics for a vehicle traversing the zone.
///
/// This rule is thought to be the entry point of a chain of rules that define
/// the properties of vehicles that can travel along a Lane. When vehicles are
/// allowed, a child rule could potentially define if motorized or non-motorized
/// could travel. In case of total restriction, no child rules are expected.
///
/// Each Lane location can be governed by at most one VehicleUsageRule.
class VehicleUsageRule : public RuleBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VehicleUsageRule);

  /// Constructs a VehicleUsageRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param states a vector of valid states for the rule
  VehicleUsageRule(
      const RuleBase::Id& id, const LaneSRange& zone,
      std::vector<std::unique_ptr<RuleState>> states)
        : RuleBase(id, zone, type(), std::move(states)) {}

  /// Maps VehicleUsageType to string representations.
  static std::unordered_map<int, const char*, drake::DefaultHash>
  StateTypeMapper();

  /// Returns a RuleTypeId initialized with this class name.
  static RuleBase::RuleTypeId type();
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
