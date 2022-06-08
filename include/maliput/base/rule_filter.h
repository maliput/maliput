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

#include <functional>
#include <vector>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"

namespace maliput {

/// Convenient alias of a functor to filter api::rules::DiscreteValueRules based
/// on arbitrary criteria from a api::rules::RoadRulebook::QueryResult.
///
/// @see FilterRules() for further information.
using DiscreteValueRuleFilter = std::function<bool(const api::rules::DiscreteValueRule&)>;

/// Convenient alias of a functor to filter api::rules::RangeValueRule based on
/// arbitrary criteria from a api::rules::RoadRulebook::QueryResult.
///
/// @see FilterRules() for further information.
using RangeValueRuleFilter = std::function<bool(const api::rules::RangeValueRule&)>;

/// Returns `rules` after the application of `discrete_value_rules_filters` and
/// `range_value_rules_filters`.
///
/// Whether a rule is preserved or not is derived from applying each functor in
/// `discrete_value_rules_filters` and `range_value_rules_filters` to their
/// respective `rules` map (by type) as a chain of logic _ands_.
api::rules::RoadRulebook::QueryResults FilterRules(
    const api::rules::RoadRulebook::QueryResults& rules,
    const std::vector<DiscreteValueRuleFilter>& discrete_value_rules_filters,
    const std::vector<RangeValueRuleFilter>& range_value_rules_filters);

}  // namespace maliput
