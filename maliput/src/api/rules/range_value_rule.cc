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
#include "maliput/api/rules/range_value_rule.h"

#include <algorithm>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

RangeValueRule::RangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                               const std::vector<Range>& ranges)
    : Rule(id, type_id, zone), states_(ranges) {
  MALIPUT_VALIDATE(!states_.empty(), "RangeValueRule(" + id.string() + ") has no RangeValueRule::Ranges.");
  for (const Range& range : states_) {
    ValidateRelatedRules(range.related_rules);
    ValidateRelatedUniqueIds(range.related_unique_ids);
    ValidateSeverity(range.severity);
    MALIPUT_VALIDATE(range.min <= range.max,
                     "RangeValueRule(" + id.string() + ") has a RangeValueRule::Ranges whose min > max.");
    MALIPUT_VALIDATE(std::count(states_.begin(), states_.end(), range) == 1,
                     "RangeValueRule(" + id.string() + ") has duplicated RangeValueRule::Ranges.");
  }
}

bool RangeValueRule::Range::operator<(const RangeValueRule::Range& other) const {
  if (severity < other.severity) {
    return true;
  } else if (severity > other.severity) {
    return false;
  } else if (description < other.description) {
    return true;
  } else if (description > other.description) {
    return false;
  } else if (min < other.min) {
    return true;
  } else if (min > other.min) {
    return false;
  } else if (max < other.max) {
    return true;
  }
  return false;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
