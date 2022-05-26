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

#include <vector>

#include "maliput/api/lane.h"

namespace maliput {
namespace routing {

/// Finds and returns sequences of lanes that go from a specified start lane to
/// a specified end lane. Only ongoing lanes are searched (adjacent lanes are
/// not). If @p start and @p end are the same lane, a sequence of one lane is
/// returned regardless of @p max_length_m.
///
/// @param start The lane at the start of the sequence.
/// @param end The lane at the end of the sequence.
/// @param max_length_m The maximum length of a sequence in meters, not
/// including @p start and @p end. The lengths of @p start and @p end are not
/// included because a vehicle may not fully traverse them. Getting from
/// @p start to @p end, however, requires a vehicle to fully traverse all
/// intermediate lanes in the sequence, which is why only the sum of their
/// lengths are included in the comparison with this upper bound.
/// @return A vector of lane sequences in which the first lane is @p start and
/// the last lane is @p end. An empty vector is returned if no sequences are
/// found.
std::vector<std::vector<const maliput::api::Lane*>> FindLaneSequences(const maliput::api::Lane* start,
                                                                      const maliput::api::Lane* end,
                                                                      double max_length_m);

}  // namespace routing
}  // namespace maliput
