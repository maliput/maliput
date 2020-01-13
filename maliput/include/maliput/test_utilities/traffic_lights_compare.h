#pragma once

#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/traffic_lights.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

// TODO(agalbachicar)  This should be replaced by a generic predicate
//                     which handles anything with operator==.

/// Predicate-formatter which tests equality of BulbColor.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbColor& a,
                                   const BulbColor& b);

/// Predicate-formatter which tests equality of BulbType.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbType& a,
                                   const BulbType& b);

/// Predicate-formatter which tests equality of BulbState.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbState& a,
                                   const BulbState& b);

/// Predicate-formatter which tests equality of std::optional<double>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const std::optional<double>& a,
                                   const std::optional<double>& b);

/// Predicate-formatter which tests equality of Bulb::BoundingBox.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Bulb::BoundingBox& a,
                                   const Bulb::BoundingBox& b);

/// Predicate-formatter which tests equality of const Bulb*.
/// Note that pointers are not evaluated but the contents they refer to are.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Bulb* a, const Bulb* b);

/// Predicate-formatter which tests equality of std::vector<const Bulb*>.
/// Note that pointers are not evaluated but the contents they refer to are.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::vector<const Bulb*>& a, const std::vector<const Bulb*>& b);

/// Predicate-formatter which tests equality of const BulbGroup*.
/// Note that pointers are not evaluated but the contents they refer to are.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbGroup* a,
                                   const BulbGroup* b);

/// Predicate-formatter which tests equality of const TrafficLight*.
/// Note that pointers are not evaluated but the contents they refer to are.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const TrafficLight* a,
                                   const TrafficLight* b);

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
