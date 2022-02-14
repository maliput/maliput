#pragma once

#include <gtest/gtest.h>

#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.

/// Predicate-formatter which tests equality of RightOfWayRule::ZoneType.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   rules::RightOfWayRule::ZoneType a, rules::RightOfWayRule::ZoneType b);

// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
/// Predicate-formatter which tests equality of RightOfWayRule::State::Type.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   rules::RightOfWayRule::State::Type a, rules::RightOfWayRule::State::Type b);

/// Predicate-formatter which tests equality of RightOfWayRule::State.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::RightOfWayRule::State& a, const rules::RightOfWayRule::State& b);
#pragma GCC diagnostic pop

/// Predicate-formatter which tests equality of
/// RightOfWayRule::RelatedBulbGroups which expands to std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& a,
                                   const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& b);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Predicate-formatter which tests equality of RightOfWayRule.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b);

/// Predicate-formatter which tests equality of
/// RuleStateProvider::RightOfWayResult.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::RightOfWayRuleStateProvider::RightOfWayResult& a,
                                   const rules::RightOfWayRuleStateProvider::RightOfWayResult& b);
#pragma GCC diagnostic pop

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
