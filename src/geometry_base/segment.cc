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
#include "maliput/geometry_base/segment.h"

namespace maliput {
namespace geometry_base {

const api::Junction* Segment::do_junction() const { return junction_; }

void Segment::AttachToJunction(common::Passkey<Junction>, const api::Junction* junction) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(junction != nullptr);
  // Preconditions
  MALIPUT_THROW_UNLESS(junction_ == nullptr);

  junction_ = junction;
}

void Segment::SetLaneIndexingCallback(common::Passkey<Junction>,
                                      const std::function<void(const api::Lane*)>& callback) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(!!callback);
  // Preconditions
  MALIPUT_THROW_UNLESS(!lane_indexing_callback_);

  lane_indexing_callback_ = callback;
  // Index any Lanes that had already been added to this Segment.
  for (const auto& lane : lanes_) {
    lane_indexing_callback_(lane.get());
  }
}

void Segment::AddLanePrivate(std::unique_ptr<Lane> lane) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(lane.get() != nullptr);
  lanes_.emplace_back(std::move(lane));
  Lane* const raw_lane = lanes_.back().get();

  raw_lane->AttachToSegment({}, this, lanes_.size() - 1);
  if (lane_indexing_callback_) {
    lane_indexing_callback_(raw_lane);
  }
}

const api::Lane* Segment::do_lane(int index) const { return lanes_.at(index).get(); }

}  // namespace geometry_base
}  // namespace maliput
