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

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "maliput/api/branch_point.h"
#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/passkey.h"
#include "maliput/geometry_base/lane_end_set.h"

namespace maliput {
namespace geometry_base {

class BranchPoint;
class Lane;
class RoadGeometry;

/// geometry_base's implementation of api::BranchPoint.
class BranchPoint : public api::BranchPoint {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(BranchPoint);

  /// Constructs an empty BranchPoint.
  ///
  /// @param id the ID of the BranchPoint
  ///
  /// The BranchPoint is not fully initialized until it is added to a
  /// RoadGeometry.
  explicit BranchPoint(const api::BranchPointId& id);

  /// Adds the @p end of @p lane to the "A side" of this BranchPoint.
  ///
  /// @throws std::exception if `lane` is nullptr.
  void AddABranch(Lane* lane, api::LaneEnd::Which end) { AddBranch(lane, end, &a_side_); }

  /// Adds the @p end of @p lane to the "B side" of this BranchPoint.
  ///
  /// @throws std::exception if `lane` is nullptr.
  void AddBBranch(Lane* lane, api::LaneEnd::Which end) { AddBranch(lane, end, &b_side_); }

  /// Sets the default branch for @p lane_end to @p default_branch.
  ///
  /// @throws std::exception if the specified LaneEnds do not belong
  ///         to opposite sides of this BranchPoint.
  void SetDefault(const api::LaneEnd& lane_end, const api::LaneEnd& default_branch);

  ~BranchPoint() override = default;

  // Notifies BranchPoint of its parent RoadGeometry.
  // This may only be called, once, by a RoadGeometry.
  //
  // @param road_geometry  the parent RoadGeometry
  //
  // @pre `road_geometry` is non-null.
  // @pre Parent RoadGeometry has not already been set.
  void AttachToRoadGeometry(common::Passkey<RoadGeometry>, const api::RoadGeometry* road_geometry);

 private:
  // Common implementation for AddABranch() and AddBBranch().
  void AddBranch(Lane* lane, api::LaneEnd::Which end, LaneEndSet* side);

  api::BranchPointId do_id() const override;

  const api::RoadGeometry* do_road_geometry() const override;

  const api::LaneEndSet* DoGetConfluentBranches(const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(const api::LaneEnd& end) const override;

  std::optional<api::LaneEnd> DoGetDefaultBranch(const api::LaneEnd& end) const override;

  const api::LaneEndSet* DoGetASide() const override;

  const api::LaneEndSet* DoGetBSide() const override;

  api::BranchPointId id_;
  const api::RoadGeometry* road_geometry_{};
  LaneEndSet a_side_;
  LaneEndSet b_side_;

  std::map<api::LaneEnd, LaneEndSet*, api::LaneEnd::StrictOrder> confluent_branches_;
  std::map<api::LaneEnd, LaneEndSet*, api::LaneEnd::StrictOrder> ongoing_branches_;
  std::map<api::LaneEnd, api::LaneEnd, api::LaneEnd::StrictOrder> defaults_;
};

}  // namespace geometry_base
}  // namespace maliput
