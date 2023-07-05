// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
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
namespace routing {

/// Defines the possible relationships the routing API can interpret between
/// two api::LaneSRanges within a Route.
/// Relations described in this enum must be mutually exclusive. The following
/// ASCII art helps to understand the relative relations described below.
///
/// <pre>
///
///  x------A>-------x
///  x------B>-------x------E>-------x------H>-------x
///  x------C>-------x------F>-------x------I>-------x
///  x------D>-------x------G>-------x
///
/// </pre>
///
/// In the diagram above:
/// - `x` indicates the beginning or the end of a api::LaneSRange.
/// - `>` indicates the direction of travel of the api::LaneSRange.
/// - The letters name the api::LaneSRanges in the Route.
///
/// Thus,
/// - `A` is LaneSRangeRelation::kCoincident with `A`.
/// - `A` is LaneSRangeRelation::kAdjacentLeft of `B`.
/// - `B` is LaneSRangeRelation::kAdjacentRight of `A`.
/// - `A` is LaneSRangeRelation::kLeft of `D`.
/// - `D` is LaneSRangeRelation::kRight of `A`.
/// - `F` is LaneSRangeRelation::kSucceedingStraight of `C`.
/// - `E` and `F` are LaneSRangeRelation::kSucceedingLeft of `D`.
/// - `F` and `G` are LaneSRangeRelation::kSucceedingRight of `B`.
/// - `E` is LaneSRangeRelation::kPreceedingStraight of `H`.
/// - `E` is LaneSRangeRelation::kPreceedingLeft of `I`.
/// - `F` and `G` are LaneSRangeRelation::kPreceedingRight of `H`.
/// - `A` is LaneSRangeRelation::kUnrelated to `H`.
///
/// LaneSRangeRelation::kUnknown represents the case when any of the
/// api::LaneSRanges is not found in the Route.
enum class LaneSRangeRelation {
  kAdjacentLeft,
  kAdjacentRight,
  kLeft,
  kRight,
  kSucceedingStraight,
  kSucceedingLeft,
  kSucceedingRight,
  kPreceedingStraight,
  kPreceedingLeft,
  kPreceedingRight,
  kCoincident,
  kUnrelated,
  kUnknown,
};

}  // namespace routing
}  // namespace maliput
