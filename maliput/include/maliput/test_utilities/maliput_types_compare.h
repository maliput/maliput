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

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"

namespace maliput {
namespace api {
namespace test {

// Compares equality within @p tolerance deviation of two InertialPosition objects.
// @param pos1 A InertialPosition object to compare.
// @param pos2 A InertialPosition object to compare.
// @param tolerance An allowable absolute deviation for each InertialPosition's
// coordinate.
// @return ::testing::AssertionFailure() When InertialPosition objects are different.
// @return ::testing::AssertionSuccess() When InertialPosition objects are within
// the @p tolerance deviation.
::testing::AssertionResult IsInertialPositionClose(const InertialPosition& pos1, const InertialPosition& pos2,
                                                   double tolerance);

// Compares equality within @p tolerance deviation of two LanePosition objects.
// @param pos1 A LanePosition object to compare.
// @param pos2 A LanePosition object to compare.
// @param tolerance An allowable absolute deviation for each LanePosition's
// coordinate.
// @return ::testing::AssertionFailure() When LanePosition objects are
// different.
// @return ::testing::AssertionSuccess() When LanePosition objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsLanePositionClose(const LanePosition& pos1, const LanePosition& pos2, double tolerance);

// Compares equality within @p tolerance deviation of two Rotation objects.
// Comparison will evaluate the inner Rotation's Euler angles.
// @param rot1 A Rotation object to compare.
// @param rot2 A Rotation object to compare.
// @param tolerance An allowable absolute deviation for each Rotation's
// coordinate.
// @return ::testing::AssertionFailure() When Rotation objects are different.
// @return ::testing::AssertionSuccess() When Rotation objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsRotationClose(const Rotation& rot1, const Rotation& rot2, double tolerance);

// Compares equality within @p tolerance deviation of two RBounds objects.
// @param rbounds1 A RBounds object to compare.
// @param rbounds2 A RBounds object to compare.
// @param tolerance An allowable absolute deviation for each RBounds's instance
// value.
// @return ::testing::AssertionFailure() When RBounds objects are different.
// @return ::testing::AssertionSuccess() When RBounds objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsRBoundsClose(const RBounds& rbounds1, const RBounds& rbounds2, double tolerance);

// Compares equality within @p tolerance deviation of two HBounds objects.
// @param hbounds1 A HBounds object to compare.
// @param hbounds1 A HBounds object to compare.
// @param tolerance An allowable absolute deviation for each HBounds's instance
// value.
// @return ::testing::AssertionFailure() When HBounds objects are different.
// @return ::testing::AssertionSuccess() When HBounds objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsHBoundsClose(const HBounds& hbounds1, const HBounds& hbounds2, double tolerance);

}  // namespace test
}  // namespace api
}  // namespace maliput
