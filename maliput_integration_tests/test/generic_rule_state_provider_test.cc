#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"

#include "maliput/api/lane.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace test {
namespace {

/// Abstract interface for the provider of the state of various rules.
class RuleStateProviderBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleStateProviderBase)

  virtual ~RuleStateProviderBase() = default;

  /// Result returned by GetState(const RuleBase::Id).
  struct RuleResult {
    /// Information about a subsequent RuleState.
    struct Next {
      /// ID of the RuleState.
      api::rules::RuleState::Id id;
      /// If known, estimated time until the transition to the RuleState.
      drake::optional<double> duration_until;
    };

    /// ID of the rule's current RuleState.
    api::rules::RuleState::Id current_id;

    /// Information about the rule's upcoming RuleState if a state transition
    /// is anticipated.
    drake::optional<Next> next;
  };

  /// Gets the state of the Rule identified by `id`.
  ///
  /// Returns a RuleResult struct bearing the State::Id of the rule's
  /// current state.  If a transition to a new state is anticipated,
  /// RuleResult::next will be populated and bear the State::Id of the
  /// next state.  If the time until the transition is known, then
  /// RuleBase::next.duration_until will be populated with that
  /// duration.
  ///
  /// Returns drake::nullopt if `id` is unrecognized, which would be the case
  /// if no such rule exists or if the rule has only static semantics.
  drake::optional<RuleResult> GetState(
      const api::rules::RuleBase::Id& id) const {
    return DoGetState(id);
  }

 protected:
  RuleStateProviderBase() = default;

 private:
  virtual drake::optional<RuleResult> DoGetState(
      const api::rules::RuleBase::Id& id) const = 0;
};

/// A trivial implementation of RuleStateProviderBase for RuleBase.
class RuleStateProvider : public RuleStateProviderBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RuleStateProvider)
  RuleStateProvider() = default;

  /// Adds a dynamic state to this provider.
  ///
  /// @throws std::logic_error if a Rule with an ID of @p id already exists in
  /// this provider.
  /// @throws std::runtime_error if the dynamic state failed to be added.
  void AddState(const api::rules::RuleBase::Id& id,
                const api::rules::RuleState::Id& initial_state_id) {
    auto result = states_.emplace(id, initial_state_id);
    if (!result.second) {
       throw std::logic_error(
          "Attempted to add multiple rules with id " + id.string());
    }
  }

  /// Sets the dynamic state of a Rule within this provider.
  ///
  /// @throws std::out_of_range if no dynamic state with @p id exists in this
  /// provider.
  void SetState(const api::rules::RuleBase::Id& id,
                const api::rules::RuleState::Id& state_id) {
    states_.at(id) = state_id;
  }

 private:
  drake::optional<RuleStateProviderBase::RuleResult> DoGetState(
      const api::rules::RuleBase::Id& id) const {
    auto it = states_.find(id);
    if (it == states_.end()) {
      return drake::nullopt;
    }
    return RuleStateProviderBase::RuleResult{it->second, drake::nullopt};
  }

  std::unordered_map<api::rules::RuleBase::Id, api::rules::RuleState::Id> states_;
};


// A simple RuleStateType implementation for testing purposes.
class MyRuleStateType : public api::rules::RuleStateType {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyRuleStateType);

  // Returns a MyRuleStateType initialized with `kA` as value and "kA" as name.
  static std::unique_ptr<MyRuleStateType> A() {
    return std::unique_ptr<MyRuleStateType>(new MyRuleStateType(kA, "kA"));
  }

  // Returns a MyRuleStateType initialized with `kB` as value and "kB" as name.
  static std::unique_ptr<MyRuleStateType> B() {
    return std::unique_ptr<MyRuleStateType>(new MyRuleStateType(kB, "kB"));
  }

 private:
  // Holds the possible types.
  enum Type {
    kA,
    kB
  };

  // Constructs a MyRuleStateType.
  //
  // @param value the value of the RuleStateType.
  // @param name the name of the RuleStateType.
  MyRuleStateType(Type value, const std::string& name) :
      api::rules::RuleStateType(static_cast<int>(value), name) {}
};

// Creates a Rule with `rule_name` and populates its state with `state_type`.
std::unique_ptr<api::rules::RuleBase> CreateRule(
    const std::string& rule_name,
    std::unique_ptr<api::rules::RuleStateType> state_type) {
  DRAKE_ASSERT(state_type != nullptr);

  const api::rules::RuleState::Id kStateId(std::string("rs_") + rule_name);
  const api::rules::RuleState::Severity kSeverity{
      api::rules::RuleState::Severity::kPreferred};

  const api::rules::RuleBase::Id kRuleId(rule_name);
  const api::rules::RuleBase::RuleTypeId kRuleTypeId(
      rule_name + std::string("_id"));
  const api::rules::LaneSRange kZone(
      api::LaneId("l_id"), api::rules::SRange(0., 100.));
  std::vector<std::unique_ptr<api::rules::RuleState>> states;
  states.push_back(std::make_unique<api::rules::RuleState>(
      kStateId, kSeverity, std::move(state_type)));

  return std::make_unique<api::rules::RuleBase>(
      kRuleId, kZone, kRuleTypeId, std::move(states));
}

// Evaluates a fresh initialized RuleStateProvider.
GTEST_TEST(RuleStateProviderTest, InitializationTest) {
  RuleStateProvider dut;
  EXPECT_EQ(dut.GetState(api::rules::RuleBase::Id("r_id")), drake::nullopt);
}

// Evaluates adding and getting states.
GTEST_TEST(RuleStateProviderTest, AddStateTest) {
  RuleStateProvider dut;

  auto rule_a = CreateRule("rule_A", MyRuleStateType::A());
  dut.AddState(rule_a->id(), rule_a->static_state().id());

  drake::optional<RuleStateProviderBase::RuleResult> result =
      dut.GetState(rule_a->id());

  EXPECT_NE(result, drake::nullopt);
  EXPECT_EQ(result->current_id, rule_a->static_state().id());
  EXPECT_EQ(result->next, drake::nullopt);
}

}  // namespace
}  // namespace test
}  // namespace maliput
