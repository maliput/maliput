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

#include <memory>
#include <optional>
#include <string>

#include "maliput/api/lane_data.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {

class RoadGeometry;

// Persistent identifier for a BranchPoint element.
using BranchPointId = TypeSpecificIdentifier<class BranchPoint>;

/// A set of LaneEnds.
class LaneEndSet {
  // NB: This is abstract (versus being a std::set or what not) to allow the
  // implementation to decide on how best to handle storage/indexing/etc.
  // E.g., it could very well be a view into a database or tiled storage or
  // something.
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LaneEndSet)

  virtual ~LaneEndSet() = default;

  /// Returns the number of LaneEnds in this set.
  ///
  /// Return value is non-negative.
  int size() const { return do_size(); }

  /// Returns the LaneEnd indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < size().
  const LaneEnd& get(int index) const { return do_get(index); }

 protected:
  LaneEndSet() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the
  /// corresponding public methods.
  ///@{
  virtual int do_size() const = 0;

  virtual const LaneEnd& do_get(int index) const = 0;
  ///@}
};

/// A BranchPoint is a node in the network of a RoadGeometry at which
/// Lanes connect to one another.  A BranchPoint is a collection of LaneEnds
/// specifying the Lanes (and, in particular, which ends of the Lanes) are
/// connected at the BranchPoint.
///
/// LaneEnds participating in a BranchPoint are grouped into two sets,
/// arbitrarily named "A-side" and "B-side".  LaneEnds on the same "side"
/// have coincident into-the-lane tangent vectors, which are anti-parallel
/// to those of LaneEnds on the other side.
class BranchPoint {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(BranchPoint)

  virtual ~BranchPoint() = default;

  /// Returns the persistent identifier.
  BranchPointId id() const { return do_id(); }

  /// Returns the RoadGeometry to which this BranchPoint belongs.
  const RoadGeometry* road_geometry() const { return do_road_geometry(); }

  /// Returns the set of LaneEnds on the same side as the given @p end,
  /// e.g., the LaneEnds merging with the given @p end.
  ///
  /// The returned set includes the given @p end.
  ///
  /// @pre @p end must be connected to the BranchPoint.
  const LaneEndSet* GetConfluentBranches(const LaneEnd& end) const { return DoGetConfluentBranches(end); }

  /// Returns the set of LaneEnds on the other side from the given @p end,
  /// e.g., the LaneEnds which @p end flows into.
  ///
  /// @pre @p end must be connected to the BranchPoint.
  const LaneEndSet* GetOngoingBranches(const LaneEnd& end) const { return DoGetOngoingBranches(end); }

  /// Returns the default ongoing branch (if any) for the given @p end.
  /// This typically represents what would be considered "continuing
  /// through-traffic" from @p end (e.g., as opposed to a branch executing
  /// a turn).
  ///
  /// If @p end has no default-branch at this BranchPoint, the return
  /// value will be std::nullopt.
  ///
  /// @pre @p end must be connected to the BranchPoint.
  std::optional<LaneEnd> GetDefaultBranch(const LaneEnd& end) const { return DoGetDefaultBranch(end); }

  /// Returns the set of LaneEnds grouped together on the "A-side".
  const LaneEndSet* GetASide() const { return DoGetASide(); }

  /// Returns the set of LaneEnds grouped together on the "B-side".
  const LaneEndSet* GetBSide() const { return DoGetBSide(); }

 protected:
  BranchPoint() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the
  /// corresponding public methods.
  ///@{
  virtual BranchPointId do_id() const = 0;

  virtual const RoadGeometry* do_road_geometry() const = 0;

  virtual const LaneEndSet* DoGetConfluentBranches(const LaneEnd& end) const = 0;

  virtual const LaneEndSet* DoGetOngoingBranches(const LaneEnd& end) const = 0;

  virtual std::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd& end) const = 0;

  virtual const LaneEndSet* DoGetASide() const = 0;

  virtual const LaneEndSet* DoGetBSide() const = 0;
  ///@}
};

}  // namespace api
}  // namespace maliput
