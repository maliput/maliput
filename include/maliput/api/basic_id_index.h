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

#include <unordered_map>

#include "maliput/api/branch_point.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {

/// Basic general-purpose concrete implementation of the
/// RoadGeometry::IdIndex interface.
class BasicIdIndex : public RoadGeometry::IdIndex {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(BasicIdIndex);

  BasicIdIndex() = default;
  ~BasicIdIndex() override = default;

  /// Adds @p lane to the index.
  ///
  /// @throws std::exception if @p lane's id() already exists in the index.
  /// @pre @p lane is not nullptr.
  void AddLane(const Lane* lane);

  /// Adds @p segment to the index.
  ///
  /// @throws std::exception if @p segment's id() already exists in the index.
  /// @pre @p segment is not nullptr.
  void AddSegment(const Segment* segment);

  /// Adds @p junction to the index.
  ///
  /// @throws std::exception if @p junction's id() already exists in the index.
  /// @pre @p junction is not nullptr.
  void AddJunction(const Junction* junction);

  /// Adds @p branch_point to the index.
  ///
  /// @throws std::exception if @p branch_point's id() already exists in the
  /// index.
  /// @pre @p branch_point is not nullptr.
  void AddBranchPoint(const BranchPoint* branch_point);

  /// Walks the object graph rooted at @p road_geometry and adds all
  /// components (Lane, Segment, Junction, BranchPoint) to the index.
  ///
  /// @throws std::exception if the graph of @p road_geometry contains any
  /// duplicate id's, or if any of its id's already exist in the index.
  /// @pre @p road_geometry is not nullptr.
  void WalkAndAddAll(const RoadGeometry* road_geometry);

 private:
  const Lane* DoGetLane(const LaneId& id) const final;
  const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override;
  const Segment* DoGetSegment(const SegmentId& id) const final;
  const Junction* DoGetJunction(const JunctionId& id) const final;
  const BranchPoint* DoGetBranchPoint(const BranchPointId& id) const final;

  std::unordered_map<JunctionId, const Junction*> junction_map_;
  std::unordered_map<SegmentId, const Segment*> segment_map_;
  std::unordered_map<LaneId, const Lane*> lane_map_;
  std::unordered_map<BranchPointId, const BranchPoint*> branch_point_map_;
};

}  // namespace api
}  // namespace maliput
