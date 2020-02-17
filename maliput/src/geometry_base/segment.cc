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
