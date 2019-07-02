#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/rules/regions.h"
#include "maliput/api/type_specific_identifier.h"

namespace maliput {
namespace api {
namespace rules {

/// Base class to define a rule state type.
///
/// This class aims to wrap enumerated literals to enable polymorphic treatment.
/// Child implementations should provide semantic construction methods.
class RuleStateType {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleStateType);
  virtual ~RuleStateType() = default;

  /// Returns the type value constant.
  int value() const { return value_; }

  /// Returns the named type.
  std::string string() const { return name_; }

 protected:
  // Constructs a RuleStateType.
  //
  // @param value an integer to represent the type.
  // @param name a string to name the type.
  RuleStateType(int value, const std::string& name) :
      value_(value), name_(name) {}

 private:
  const int value_{};
  const std::string name_;
};

/// Base class to define a rule state.
///
/// Child implementations are expected to work together with child
/// implementations of RuleStateType and evaluate type validity at construct
/// time.
class RuleState {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleState);

  using Id = TypeSpecificIdentifier<class RuleState>;

  /// RuleState severity.
  ///
  /// Determines the enforcement of this rule state.
  enum class Severity {
    /// Rule state must not be transgressed.
    kStrict = 0,
    /// Rule state enforcement is recommended.
    kPreferred,
  };

  RuleState() = delete;

  /// Constructs a RuleState instance.
  ///
  /// @param id the unique Id
  /// @param severity the Severity of the State
  /// @param type the semantic Type. It must not be nullptr.
  RuleState(const Id& id, Severity severity,
            std::unique_ptr<RuleStateType> type)
      : id_(id), severity_(severity), type_(std::move(type)) {
    DRAKE_DEMAND(type_ != nullptr);
  }

  virtual ~RuleState() = default;

  /// Returns the Id.
  const Id& id() const { return id_; }

  /// Returns the Type.
  const RuleStateType* type() const { return type_.get(); }

  /// Returns the Severity.
  Severity severity() const { return severity_; }

 protected:
  const Id id_;
  const Severity severity_{};
  std::unique_ptr<RuleStateType> type_;
};

// TODO(andrew.best@tri.global): Add support for multiple states. Currently,
// it's enforced that all rules have exactly one state and are therefore
// static. To support multiple states, a StateProvider is needed.
/// Base class to describe a Rule for a RoadRuleBook.
class RuleBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleBase);

  using Id = TypeSpecificIdentifier<class RuleBase>;
  using RuleTypeId = TypeSpecificIdentifier<std::string>;

  /// Constructs a Rule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param states a vector of valid states for the rule
  RuleBase(const Id& id, const LaneSRange& zone,
           const RuleTypeId& rule_type,
           std::vector<std::unique_ptr<RuleState>> states)
      : id_(id), zone_(zone), rule_type_(rule_type) {
    DRAKE_THROW_UNLESS(states.size() == 1);
    for (int i = 0; i < states.size(); ++i) {
      DRAKE_THROW_UNLESS(states.at(i) != nullptr);
      // Construct index of states by ID, ensuring uniqueness of ID's.
      auto result = states_.emplace(states.at(i)->id(), std::move(states.at(i)));
      DRAKE_THROW_UNLESS(result.second);
    }
  }

  virtual ~RuleBase() = default;

  /// Returns the persistent identifier.
  Id id() const { return id_; }

  /// Returns the zone to which this rule instance applies.
  const LaneSRange& zone() const { return zone_; }

  /// Returns the child-rule type.
  RuleTypeId rule_type() const { return rule_type_; }

  /// Returns the catalog of possible States.
  const std::unordered_map<RuleState::Id, std::unique_ptr<RuleState>>&
  states() const {
    return states_;
  }

  /// Returns true if the rule is static, i.e. has only one state,
  /// otherwise false.
  bool is_static() const { return states_.size() == 1; }

  /// Returns the static state of the rule.
  ///
  /// This is a convenience function for returning a static rule's single state.
  ///
  /// @throws std::exception if `is_static()` is false.
  const RuleState& static_state() const {
    DRAKE_THROW_UNLESS(is_static());
    return *(states_.begin()->second);
  }

 private:
  Id id_;
  LaneSRange zone_;
  RuleTypeId rule_type_;
  std::unordered_map<RuleState::Id, std::unique_ptr<RuleState>> states_;
};

/// Holds a group of related rules that impose a collection of restrictions.
///
/// Rules might have a complete meaning on their own or might need of the
/// previous or posterior constraint to better describe more complex
/// restrictions. Rules in the context of a RuleGroup must uniquely define a
/// semantic restriction that is coherent in terms of regions and RuleState
/// evolution.
class RuleGroup {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleGroup);

  using Id = TypeSpecificIdentifier<class RuleGroup>;

  /// Constructs a RuleGroup.
  ///
  /// @param id is the unique ID of this group (in the RoadRulebook)
  /// @param rules is a non-empty vector with related rules.
  /// @throws std::runtime_error when `rules` is empty, any rule is nullptr or
  ///         rules' zones are different.
  RuleGroup(const Id& id, std::vector<std::unique_ptr<RuleBase>> rules) :
      id_(id), rules_(std::move(rules)) {
    DRAKE_THROW_UNLESS(!rules_.empty());
    for (int i = 0; i < rules_.size(); ++i) {
      DRAKE_THROW_UNLESS(rules_.at(i) != nullptr);
    }
    const LaneSRange lane_s_range_0 = rules_.at(0)->zone();
    for (int i = 1; i < rules_.size(); ++i) {
      DRAKE_THROW_UNLESS(
          AreLaneSRangesEqual(lane_s_range_0, rules_.at(1)->zone()));
    }
  }

  /// Returns the ID.
  Id id() const { return id_; }

  /// Returns number of rules this group holds.
  int size() const { return static_cast<int>(rules_.size()); }

  /// Returns a pointer to the @p index-th rule.
  /// @throws std::runtime_error when index is negative or greater or equal to
  ///         size();
  RuleBase* rule(int index) const {
    DRAKE_THROW_UNLESS(index >= 0 && index < size());
    return rules_.at(index).get();
  }

 private:
  // Compares two LaneSRanges.
  //
  // @param lane_s_range_0 a LaneSRange.
  // @param lane_s_range_1 a LaneSRange.
  // @return true When `lane_s_range_0` is equal to `lane_s_range_1`.
  bool AreLaneSRangesEqual(const LaneSRange& lane_s_range_0,
                          const LaneSRange& lane_s_range_1) const {
    return lane_s_range_0.lane_id() == lane_s_range_1.lane_id() &&
           lane_s_range_0.s_range().s0() == lane_s_range_1.s_range().s0() &&
           lane_s_range_0.s_range().s1() == lane_s_range_1.s_range().s1();
  }

  Id id_;
  std::vector<std::unique_ptr<RuleBase>> rules_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
