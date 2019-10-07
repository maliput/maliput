#include "maliput/geometry_base/junction.h"

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace geometry_base {

void Junction::AttachToRoadGeometry(common::Passkey<RoadGeometry>, const api::RoadGeometry* road_geometry,
                                    const std::function<void(const api::Segment*)>& segment_indexing_callback,
                                    const std::function<void(const api::Lane*)>& lane_indexing_callback) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(!!segment_indexing_callback);
  MALIPUT_THROW_UNLESS(!!lane_indexing_callback);
  // Preconditions
  MALIPUT_THROW_UNLESS(road_geometry_ == nullptr);
  MALIPUT_THROW_UNLESS(!segment_indexing_callback_);
  MALIPUT_THROW_UNLESS(!lane_indexing_callback_);

  road_geometry_ = road_geometry;
  // Store the indexing callbacks for future use.
  segment_indexing_callback_ = segment_indexing_callback;
  lane_indexing_callback_ = lane_indexing_callback;

  // Index any Segments that had already been added to this Junction.
  for (auto& segment : segments_) {
    segment_indexing_callback_(segment.get());
    segment->SetLaneIndexingCallback({}, lane_indexing_callback_);
  }
}

void Junction::AddSegmentPrivate(std::unique_ptr<Segment> segment) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(segment.get() != nullptr);
  segments_.emplace_back(std::move(segment));
  Segment* const raw_segment = segments_.back().get();

  raw_segment->AttachToJunction({}, this);
  if (segment_indexing_callback_) {
    segment_indexing_callback_(raw_segment);
  }
  if (lane_indexing_callback_) {
    raw_segment->SetLaneIndexingCallback({}, lane_indexing_callback_);
  }
}

const api::RoadGeometry* Junction::do_road_geometry() const { return road_geometry_; }

}  // namespace geometry_base
}  // namespace maliput
