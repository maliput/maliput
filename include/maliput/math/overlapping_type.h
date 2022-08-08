// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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

namespace maliput {
namespace math {

/// Holds the possible overlapping types between regions.
///
/// Given two sets `A` and `B` :
/// - `A` intersects `B` iff `A` and `B` have at least one point in common.
/// - `A` contains `B` iff `A` contains all the points of `B`.
/// - `A` disjoints `B` iff `A` and `B` have no points in common.
///
///  - Example of use:
/// @code {.cpp}
/// OverlappingType MyMethod();
/// ...
/// if(OverlappingType::kIntersected & MyMethod() == OverlappingType::kIntersected) {
///  // Do something.
/// }
/// @endcode
enum class OverlappingType : unsigned int {
  kDisjointed = 0,   ///< No overlapping between bounding regions
  kIntersected = 1,  ///< Bounding regions are intersected.
  kContained = 3,    ///< Bounding regions are contained.
};

// Union operator.
OverlappingType operator|(const OverlappingType& first, const OverlappingType& second);

// Intersection operator.
OverlappingType operator&(const OverlappingType& first, const OverlappingType& second);

}  // namespace math
}  // namespace maliput
