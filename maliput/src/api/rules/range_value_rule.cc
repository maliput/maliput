#include "maliput/api/rules/range_value_rule.h"

#include <algorithm>

namespace maliput {
namespace api {
namespace rules {

RangeValueRule::RangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                               const std::vector<Range>& ranges)
    : Rule(id, type_id, zone), ranges_(ranges) {
  MALIPUT_THROW_UNLESS(!ranges_.empty());
  for (const Range& range : ranges_) {
    ValidateRelatedRules(range.related_rules);
    ValidateSeverity(range.severity);
    MALIPUT_THROW_UNLESS(range.min <= range.max);
    MALIPUT_THROW_UNLESS(std::count(ranges_.begin(), ranges_.end(), range) == 1);
  }
}

RangeValueRule::Range MakeRange(int severity, const Rule::RelatedRules& related_rules, const std::string& description,
                                double min, double max) {
  RangeValueRule::Range range;
  range.severity = severity;
  range.related_rules = related_rules;
  range.description = description;
  range.min = min;
  range.max = max;
  return range;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
