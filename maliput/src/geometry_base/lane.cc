#include "maliput/geometry_base/lane.h"

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/geometry_base/branch_point.h"

namespace maliput {
namespace geometry_base {

void Lane::AttachToSegment(common::Passkey<Segment>, const api::Segment* segment, int index) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(segment != nullptr);
  MALIPUT_THROW_UNLESS(index >= 0);
  // Preconditions
  MALIPUT_THROW_UNLESS(segment_ == nullptr);
  MALIPUT_THROW_UNLESS(index_ == -1);

  segment_ = segment;
  index_ = index;
}

void Lane::SetStartBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(branch_point != nullptr);
  // Preconditions
  MALIPUT_THROW_UNLESS(start_branch_point_ == nullptr);

  start_branch_point_ = branch_point;
}

void Lane::SetFinishBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(branch_point != nullptr);
  // Preconditions
  MALIPUT_THROW_UNLESS(finish_branch_point_ == nullptr);

  finish_branch_point_ = branch_point;
}

api::LaneId Lane::do_id() const { return id_; }

const api::Segment* Lane::do_segment() const { return segment_; }

int Lane::do_index() const { return index_; }

const api::Lane* Lane::do_to_left() const {
  if (index_ == (segment_->num_lanes() - 1)) {
    return nullptr;
  } else {
    return segment_->lane(index_ + 1);
  }
}

const api::Lane* Lane::do_to_right() const {
  if (index_ == 0) {
    return nullptr;
  } else {
    return segment_->lane(index_ - 1);
  }
}

const api::BranchPoint* Lane::DoGetBranchPoint(const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart: {
      return start_branch_point_;
    }
    case api::LaneEnd::kFinish: {
      return finish_branch_point_;
    }
  }
  MALIPUT_ABORT_MESSAGE("which_end is neither LaneEnd::kStart nor LaneEnd::kFinish.");
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetOngoingBranches({this, which_end});
}

std::optional<api::LaneEnd> Lane::DoGetDefaultBranch(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}

api::InertialPosition Lane::DoToInertialPosition(const api::LanePosition& lane_pos) const {
  const api::InertialPosition backend_inertial_position = DoToBackendInertialPosition(lane_pos);
  return api::InertialPosition::FromXyz(backend_inertial_position.xyz() +
                                        segment()->junction()->road_geometry()->internal_inertial_frame_translation());
}

api::InertialPosition Lane::DoToBackendInertialPosition(const api::LanePosition& lane_pos) const {
  MALIPUT_THROW_MESSAGE(
      "Unimplemented method. Please check the documentation of "
      "maliput::geometry_base::Lane::DoToInertialPosition().");
}

api::LanePositionResult Lane::DoToLanePosition(const api::InertialPosition& inertial_pos) const {
  const api::InertialPosition backend_inertial_position = api::InertialPosition::FromXyz(
      inertial_pos.xyz() - segment()->junction()->road_geometry()->internal_inertial_frame_translation());
  return DoToBackendLanePosition(backend_inertial_position);
}

api::LanePositionResult Lane::DoToBackendLanePosition(const api::InertialPosition& inertial_pos) const {
  MALIPUT_THROW_MESSAGE(
      "Unimplemented method. Please check the documentation of "
      "maliput::geometry_base::Lane::DoToLanePosition().");
}

}  // namespace geometry_base
}  // namespace maliput
