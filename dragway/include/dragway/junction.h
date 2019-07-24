#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/junction.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_abort.h"

#include "dragway/segment.h"


namespace maliput {
namespace dragway {

class RoadGeometry;

/// Dragway's implementation of api::Junction.
class Junction final : public api::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction)

  /// Constructs a Junction with a single Segment.
  ///
  /// @p road_geometry must remain valid for the lifetime of this class,
  /// and must refer to the RoadGeometry which will contain this newly
  /// constructed Junction instance.
  Junction(RoadGeometry* road_geometry,
      int num_lanes,
      double length,
      double lane_width,
      double shoulder_width,
      double maximum_height);

  ~Junction() final = default;

 private:
  api::JunctionId do_id() const final { return id_; }

  const api::RoadGeometry* do_road_geometry() const final;

  int do_num_segments() const final { return 1; }

  const api::Segment* do_segment(int index) const final {
    MALIPUT_DEMAND(index < num_segments());
    return &segment_;
  }

  const api::JunctionId id_;
  const RoadGeometry* const road_geometry_{};
  const Segment segment_;
};

}  // namespace dragway
}  // namespace maliput
