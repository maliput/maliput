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

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/passkey.h"
#include "maliput/geometry_base/lane.h"
#include "maliput/geometry_base/lane_boundary.h"

namespace maliput {
namespace geometry_base {

class Junction;

/// geometry_base's implementation of api::Segment.
class Segment : public api::Segment {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Segment);

  /// Constructs a new Segment.
  ///
  /// @param id Segment's ID.
  ///
  /// The Segment is not fully initialized until it is added to a Junction.
  explicit Segment(const api::SegmentId& id) : id_(id) {}

  /// Adds @p lane to this Segment.
  ///
  /// This Segment will take ownership of `lane` and will be assigned
  /// as its parent.  The index of `lane` will also be assigned, in
  /// order of addition.  (The first Lane added to this Segment, by
  /// the first call to AddLane(), will be assigned index 0.)
  ///
  /// Note that the maliput API requires that lanes are indexed in a Segment
  /// in right-to-left order, thus `AddLane()` should be called on lanes
  /// in right-to-left order.
  ///
  /// @returns `lane`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::Lane.
  ///
  /// @throws std::exception if `lane` is empty.
  // TODO(maddog@tri.global)  Consider enforcing right-to-left order.  (The
  //                          downside is that it makes these classes less
  //                          useful for creating aberrant test cases.)
  template <class T>
  T* AddLane(std::unique_ptr<T> lane) {
    static_assert(std::is_base_of<Lane, T>::value, "T is not derived from geometry_base::Lane");
    T* const raw_pointer = lane.get();
    AddLanePrivate(std::move(lane));
    return raw_pointer;
  }

  /// Adds @p boundary to this Segment.
  ///
  /// This Segment will take ownership of `boundary` and will be assigned
  /// as its parent.  The index of `boundary` will also be assigned, in
  /// order of addition.  (The first LaneBoundary added to this Segment, by
  /// the first call to AddBoundary(), will be assigned index 0.)
  ///
  /// Note that the maliput API requires that lanes are indexed in a Segment
  /// in right-to-left order, thus `AddBoundary()` should be called on boundaries
  /// in right-to-left order.
  ///
  /// @returns `boundary`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::LaneBoundary.
  ///
  /// @throws std::exception if `boundary` is empty.
  template <class T>
  T* AddBoundary(std::unique_ptr<T> boundary) {
    static_assert(std::is_base_of<LaneBoundary, T>::value, "T is not derived from geometry_base::LaneBoundary");
    T* const raw_pointer = boundary.get();
    AddBoundaryPrivate(std::move(boundary));
    return raw_pointer;
  }

  // Notifies Segment of its parent Junction.
  // This may only be called, once, by a Junction.
  //
  // @param junction  the parent Junction
  //
  // @pre `junction` is non-null.
  // @pre Parent Junction has not already been set.
  void AttachToJunction(common::Passkey<Junction>, const api::Junction* junction);

  // Sets the lane indexing callback.
  // This may only be called, once, by a Junction.
  //
  // @param callback  function to be called on every Lane which is attached
  //                  to this Segment, both now and in the future
  //
  // @pre `callback` is non-empty.
  // @pre The lane indexing callback has not already been set.
  void SetLaneIndexingCallback(common::Passkey<Junction>, const std::function<void(const api::Lane*)>& callback);

  // Sets the lane boundary indexing callback.
  // This may only be called, once, by a Junction.
  //
  // @param callback  function to be called on every LaneBoundary which is attached
  //                  to this Segment, both now and in the future.
  //
  // @pre `callback` is non-empty.
  // @pre The lane boundary indexing callback has not already been set.
  void SetLaneBoundaryIndexingCallback(common::Passkey<Junction>,
                                       const std::function<void(const api::LaneBoundary*)>& callback);

  ~Segment() override = default;

 private:
  // The non-template implementation of AddLane<T>()
  void AddLanePrivate(std::unique_ptr<Lane> lane);

  // The non-template implementation of AddBoundary<T>()
  void AddBoundaryPrivate(std::unique_ptr<LaneBoundary> lane_boundary);

  api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return lanes_.size(); }

  const api::Lane* do_lane(int index) const override;

  const api::LaneBoundary* do_boundary(int index) const override {
    if (index < 0 || index >= static_cast<int>(boundaries_.size())) {
      return nullptr;
    }
    return boundaries_[index].get();
  }

  int do_num_boundaries() const override { return static_cast<int>(boundaries_.size()); }

  const api::SegmentId id_;
  const api::Junction* junction_{};
  std::function<void(const api::Lane*)> lane_indexing_callback_;
  std::vector<std::unique_ptr<Lane>> lanes_;
  std::function<void(const api::LaneBoundary*)> lane_boundary_indexing_callback_;
  // Lane boundaries for this segment.
  std::vector<std::unique_ptr<LaneBoundary>> boundaries_;
};

}  // namespace geometry_base
}  // namespace maliput
