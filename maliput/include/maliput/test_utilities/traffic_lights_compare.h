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
