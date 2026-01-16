// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include "maliput/api/lane_boundary.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/passkey.h"

namespace maliput {
namespace geometry_base {

class Segment;

/// geometry_base's implementation of api::LaneBoundary.
class LaneBoundary : public api::LaneBoundary {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LaneBoundary);

  /// Constructs a LaneBoundary.
  ///
  /// @param id the ID of the LaneBoundary
  ///
  /// The LaneBoundary is not fully initialized until it is added to a Segment.
  explicit LaneBoundary(const api::LaneBoundary::Id& id) : id_(id) {}

  // Notifies LaneBoundary of its parent Segment.
  // This may only be called, once, by a Segment.
  //
  // @param segment  the parent Segment
  // @param index  index of this LaneBoundary within `segment`
  //
  // @pre `segment` is non-null.
  // @pre `index` is non-negative.
  // @pre Parent Segment and the index have not already been set.
  void AttachToSegment(common::Passkey<Segment>, const api::Segment* segment, int index);

  ~LaneBoundary() override = default;

 private:
  api::LaneBoundary::Id do_id() const override { return id_; }
  const api::Segment* do_segment() const override { return segment_; }
  int do_index() const override { return index_; }
  const api::Lane* do_lane_to_left() const override { return lane_to_left_; }
  const api::Lane* do_lane_to_right() const override { return lane_to_right_; }
  virtual std::optional<api::LaneMarkingResult> DoGetMarking(double s) const override;
  virtual std::vector<api::LaneMarkingResult> DoGetMarkings() const override;
  virtual std::vector<api::LaneMarkingResult> DoGetMarkings(double s_start, double s_end) const override;

  const api::LaneBoundary::Id id_;
  const api::Segment* segment_{};
  int index_{-1};
  const maliput::api::Lane* lane_to_left_{};
  const maliput::api::Lane* lane_to_right_{};
};

}  // namespace geometry_base
}  // namespace maliput
