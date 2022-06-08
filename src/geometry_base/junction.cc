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
#include "maliput/geometry_base/junction.h"

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace geometry_base {

void Junction::AttachToRoadGeometry(common::Passkey<RoadGeometry>, const api::RoadGeometry* road_geometry,
                                    const std::function<void(const api::Segment*)>& segment_indexing_callback,
                                    const std::function<void(const api::Lane*)>& lane_indexing_callback) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(!!segment_indexing_callback);
  MALIPUT_THROW_UNLESS(!!lane_indexing_callback);
  // Preconditions
  MALIPUT_THROW_UNLESS(road_geometry_ == nullptr);
  MALIPUT_THROW_UNLESS(!segment_indexing_callback_);
  MALIPUT_THROW_UNLESS(!lane_indexing_callback_);

  road_geometry_ = road_geometry;
  // Store the indexing callbacks for future use.
  segment_indexing_callback_ = segment_indexing_callback;
  lane_indexing_callback_ = lane_indexing_callback;

  // Index any Segments that had already been added to this Junction.
  for (auto& segment : segments_) {
    segment_indexing_callback_(segment.get());
    segment->SetLaneIndexingCallback({}, lane_indexing_callback_);
  }
}

void Junction::AddSegmentPrivate(std::unique_ptr<Segment> segment) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(segment.get() != nullptr);
  segments_.emplace_back(std::move(segment));
  Segment* const raw_segment = segments_.back().get();

  raw_segment->AttachToJunction({}, this);
  if (segment_indexing_callback_) {
    segment_indexing_callback_(raw_segment);
  }
  if (lane_indexing_callback_) {
    raw_segment->SetLaneIndexingCallback({}, lane_indexing_callback_);
  }
}

const api::RoadGeometry* Junction::do_road_geometry() const { return road_geometry_; }

}  // namespace geometry_base
}  // namespace maliput
