#include "maliput/geometry_base/filter_positions.h"

#include <algorithm>

namespace maliput {
namespace geometry_base {

std::vector<api::RoadPositionResult> FilterRoadPositionResults(
    const std::vector<api::RoadPositionResult>& road_position_results,
    const std::function<bool(const api::RoadPositionResult&)>& filter) {
  std::vector<api::RoadPositionResult> result(road_position_results);
  // TODO(agalbachicar)   If this code is ever moved to C++17, use std::not_fn() instead.
  result.erase(std::remove_if(result.begin(), result.end(), std::not1(filter)), result.end());
  return result;
}

}  // namespace geometry_base
}  // namespace maliput
