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

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "maliput/api/junction.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/passkey.h"
#include "maliput/geometry_base/segment.h"

namespace maliput {
namespace geometry_base {

class RoadGeometry;

/// geometry_base's implementation of api::Junction.
class Junction : public api::Junction {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Junction);

  /// Constructs a Junction.
  ///
  /// @param id the ID of the Junction
  ///
  /// The Junction is not fully initialized until it is added to a
  /// RoadGeometry.
  explicit Junction(const api::JunctionId& id) : id_(id) {}

  /// Adds @p segment to this Junction.
  ///
  /// This Junction will take ownership of `segment` and will be assigned
  /// as its parent.
  ///
  /// @returns `segment`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::Segment.
  ///
  /// @throws std::exception if `segment` is empty.
  template <class T>
  T* AddSegment(std::unique_ptr<T> segment) {
    static_assert(std::is_base_of<Segment, T>::value, "T is not derived from geometry_base::Segment");
    T* const raw_pointer = segment.get();
    AddSegmentPrivate(std::move(segment));
    return raw_pointer;
  }

  ~Junction() override = default;

  // Notifies Junction of its parent RoadGeometry.
  // This may only be called, once, by a RoadGeometry.
  //
  // @param road_geometry  the parent RoadGeometry
  // @param segment_indexing_callback  function to be called on every Segment
  //                                   which is attached to this Junction,
  //                                   both now and in the future
  // @param lane_indexing_callback  function to be called on every Lane
  //                                which is attached to this Junction (via
  //                                a Segment), both now and in the future
  //
  // @pre `road_geometry` is non-null.
  // @pre `segment_indexing_callback` is non-empty.
  // @pre `lane_indexing_callback` is non-empty.
  // @pre Parent RoadGeometry and the callbacks have not already been set.
  void AttachToRoadGeometry(common::Passkey<RoadGeometry>, const api::RoadGeometry* road_geometry,
                            const std::function<void(const api::Segment*)>& segment_indexing_callback,
                            const std::function<void(const api::Lane*)>& lane_indexing_callback);

 private:
  // The non-template implementation of AddSegment<T>()
  void AddSegmentPrivate(std::unique_ptr<Segment> segment);

  api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return segments_.size(); }

  const api::Segment* do_segment(int index) const override { return segments_.at(index).get(); }

  const api::JunctionId id_;
  const api::RoadGeometry* road_geometry_{};
  std::function<void(const api::Segment*)> segment_indexing_callback_;
  std::function<void(const api::Lane*)> lane_indexing_callback_;
  std::vector<std::unique_ptr<Segment>> segments_;
};

}  // namespace geometry_base
}  // namespace maliput
