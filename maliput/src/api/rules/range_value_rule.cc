#include "maliput/api/rules/range_value_rule.h"

#include <algorithm>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

RangeValueRule::RangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                               const std::vector<Range>& ranges)
    : Rule(id, type_id, zone), ranges_(ranges) {
  MALIPUT_VALIDATE(!ranges_.empty(), "RangeValueRule(" + id.string() + ") has no RangeValueRule::Ranges.");
  for (const Range& range : ranges_) {
    ValidateRelatedRules(range.related_rules);
    ValidateRelatedUniqueIds(range.related_unique_ids);
    ValidateSeverity(range.severity);
    MALIPUT_VALIDATE(range.min <= range.max,
                     "RangeValueRule(" + id.string() + ") has a RangeValueRule::Ranges whose min > max.");
    MALIPUT_VALIDATE(std::count(ranges_.begin(), ranges_.end(), range) == 1,
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
