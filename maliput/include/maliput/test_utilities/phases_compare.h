#pragma once

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/traffic_lights.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

/// Predicate-formatter which tests equality of RuleStates.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const RuleStates& a,
                                   const RuleStates& b);

/// Predicate-formatter which tests equality of drake::optional<BulbStates>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const drake::optional<BulbStates>& a, const drake::optional<BulbStates>& b);

/// Predicate-formatter which tests equality of Phase.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Phase& a, const Phase& b);

/// Predicate-formatter which tests equality of PhaseRing::NextPhase.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const PhaseRing::NextPhase& a,
                                   const PhaseRing::NextPhase& b);

/// Predicate-formatter which tests equality of
/// std::vector<PhaseRing::NextPhase>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::vector<PhaseRing::NextPhase>& a,
                                   const std::vector<PhaseRing::NextPhase>& b);

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
