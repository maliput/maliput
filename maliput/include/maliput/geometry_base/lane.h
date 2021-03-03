#pragma once

#include <optional>

#include "maliput/api/branch_point.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/passkey.h"

namespace maliput {
namespace geometry_base {

class BranchPoint;
class Segment;

/// geometry_base's implementation of api::Lane.
class Lane : public api::Lane {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Lane);

  /// Constructs a Lane.
  ///
  /// @param id the ID of the Lane
  ///
  /// The Lane is not fully initialized until it is added to a Segment
  /// and each end is added to (one or two) BranchPoints.
  ///
  /// The Lane's index will be assigned when it is added to a Segment.
  explicit Lane(const api::LaneId& id) : id_(id) {}

  /// Returns a mutable pointer to the BranchPoint at the start end,
  /// or nullptr if the start end hasn't been added to a BranchPoint yet.
  BranchPoint* mutable_start_branch_point() { return start_branch_point_; }

  /// Returns a mutable pointer to the BranchPoint at the finish end,
  /// or nullptr if the finish end hasn't been added to a BranchPoint yet.
  BranchPoint* mutable_finish_branch_point() { return finish_branch_point_; }

  // Notifies Lane of its parent Segment.
  // This may only be called, once, by a Segment.
  //
  // @param segment  the parent Segment
  // @param index  index of this Lane within `segment`
  //
  // @pre `segment` is non-null.
  // @pre `index` is non-negative.
  // @pre Parent Segment and the index have not already been set.
  void AttachToSegment(common::Passkey<Segment>, const api::Segment* segment, int index);

  // Notifies Lane of the BranchPoint for its "start" end.
  // This may only be called, once, by a BranchPoint.
  //
  // @param branch_point  the BranchPoint
  //
  // @pre `branch_point` is non-null.
  // @pre The "start" BranchPoint has not already been set.
  void SetStartBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point);

  // Notifies Lane of the BranchPoint for its "finish" end.
  // This may only be called, once, by a BranchPoint.
  //
  // @param branch_point  the BranchPoint
  //
  // @pre `branch_point` is non-null.
  // @pre The "finish" BranchPoint has not already been set.
  void SetFinishBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point);

  ~Lane() override = default;

 private:
  api::LaneId do_id() const override;

  const api::Segment* do_segment() const override;

  int do_index() const override;

  const api::Lane* do_to_left() const override;

  const api::Lane* do_to_right() const override;

  const api::BranchPoint* DoGetBranchPoint(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetConfluentBranches(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(const api::LaneEnd::Which which_end) const override;

  std::optional<api::LaneEnd> DoGetDefaultBranch(const api::LaneEnd::Which which_end) const override;

  // @{
  // Forwards the call to DoToBackendInertialPosition(lane_pos) and translates
  // the result to account for api::RoadGeometry::internal_inertial_frame_translation().
  //
  // @note Backend implementations should not override this method and should
  //       override DoToBackendInertialPosition() when making use of this frame
  //       conversion.
  // @note Backend implementations should override this method and should not
  //       override DoToBackendInertialPostion() when they prefer to handle the
  //       entire mapping process.
  virtual api::InertialPosition DoToInertialPosition(const api::LanePosition& lane_pos) const override;

  // Maps @p lane_pos into the backend (maliput's) Inertial Frame which does not
  // include api::RoadGeometry::internal_inertial_frame_translation()
  // translation.
  //
  // This method implementation @throws maliput::common::assertion_error as it
  // must never be called as is. @see DoToInertialPosition() docstring to
  // understand when to override each method accordingly.
  virtual api::InertialPosition DoToBackendInertialPosition(const api::LanePosition& lane_pos) const;
  // @}

  // @{
  // Translates @p inertial_pos with api::RoadGeometry::internal_inertial_frame_translation()
  // and forwards the result to DoToBackendLanePosition().
  //
  // @note Backend implementations should not override this method and should
  //       override DoToBackendLanePosition() when making use of this frame
  //       conversion.
  // @note Backend implementations should override this method and should not
  //       override DoToBackendLanePosition() when they prefer to handle the
  //       entire mapping process.
  virtual api::LanePositionResult DoToLanePosition(const api::InertialPosition& inertial_pos) const override;

  // Maps @p inertial_pos (measured in the backend (maliput's) Inertial Frame
  // which does not include api::RoadGeometry::internal_inertial_frame_translation()
  // translation) into this Lane Frame.
  //
  // This method implementation @throws maliput::common::assertion_error as it
  // must never be called as is. @see DoToLanePosition() docstring to
  // understand when to override each method accordingly.
  virtual api::LanePositionResult DoToBackendLanePosition(const api::InertialPosition& inertial_pos) const;
  // @}

  const api::LaneId id_;
  const api::Segment* segment_{};
  int index_{-1};
  BranchPoint* start_branch_point_{};
  BranchPoint* finish_branch_point_{};
};

}  // namespace geometry_base
}  // namespace maliput
