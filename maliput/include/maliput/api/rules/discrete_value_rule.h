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
#pragma once

#include <string>
#include <vector>

#include "maliput/api/regions.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// Describes a discrete value rule.
///
/// DiscreteValues are defined by a string value.
/// Semantics of this rule are based on _all_ possible values that this
/// Rule::TypeId could have (as specified by RuleRegistry::FindRuleByType()),
/// not only the subset of values that a specific instance of this rule can
/// be in.
class DiscreteValueRule : public Rule {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteValueRule);

  /// Defines a discrete value for a DiscreteValueRule.
  struct DiscreteValue : public Rule::State {
    MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteValue);

    /// Default constructor.
    DiscreteValue() = default;

    /// Creates a Rule::State.
    /// @param severity A non-negative quantity that specifies the
    ///                 level of enforcement. The smaller it is, the
    ///                 more strictly the rule is enforced.
    /// @param related_rules Contains groups of related rules.
    /// @param related_unique_ids Contains groups of related unique ids.
    /// @param value Is the discrete value contained in the rule.
    DiscreteValue(int severity, RelatedRules related_rules, RelatedUniqueIds related_unique_ids, std::string value)
        : Rule::State(severity, related_rules, related_unique_ids), value(value) {}

    bool operator==(const DiscreteValue& other) const { return value == other.value && Rule::State::operator==(other); }
    bool operator!=(const DiscreteValue& other) const { return !(*this == other); }

    std::string value;  ///< Value of the DiscreteValue.
  };

  /// Constructs a DiscreteValueRule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param values A vector of possible discrete values that this Rule could be
  ///               in. The actual value that's enforced at any given time is
  ///               determined by a DiscreteValueRuleStateProvider.  It must
  ///               have at least one value; each value must be unique.
  /// @throws maliput::common::assertion_error When `values` is empty.
  /// @throws maliput::common::assertion_error When there are duplicated values
  ///         in `values`.
  DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                    const std::vector<DiscreteValue>& values);

  const std::vector<DiscreteValue>& states() const { return states_; }

 private:
  std::vector<DiscreteValue> states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
