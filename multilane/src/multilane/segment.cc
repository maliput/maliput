#include "multilane/segment.h"

namespace maliput {
namespace multilane {

const api::Junction* Segment::do_junction() const { return junction_; }

Lane* Segment::NewLane(api::LaneId id, double r0, const api::RBounds& lane_bounds) {
  const int index = lanes_.size();
  MALIPUT_DEMAND(r_min_ <= r0 && r0 <= r_max_);
  if (lanes_.size() != 0) {
    MALIPUT_DEMAND(r0 > lanes_.back()->r0());
  }
  const api::RBounds segment_bounds(r_min_ - r0, r_max_ - r0);
  MALIPUT_DEMAND(lane_bounds.min() >= segment_bounds.min() && lane_bounds.max() <= segment_bounds.max());
  auto lane_ =
      std::make_unique<Lane>(id, this, index, lane_bounds, segment_bounds, elevation_bounds_, road_curve_.get(), r0);
  lanes_.push_back(std::move(lane_));
  Lane* result = lanes_.back().get();
  register_lane_(result);
  return result;
}

const api::Lane* Segment::do_lane(int index) const {
  MALIPUT_DEMAND(index >= 0 && index < static_cast<int>(lanes_.size()));
  return lanes_[index].get();
}

}  // namespace multilane
}  // namespace maliput
