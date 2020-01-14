#include "maliput/geometry_base/filter_positions.h"

#include <algorithm>
#include <functional>

namespace maliput {
namespace geometry_base {

std::vector<api::RoadPositionResult> FilterRoadPositionResults(
    const std::vector<api::RoadPositionResult>& road_position_results,
    const std::function<bool(const api::RoadPositionResult&)>& filter) {
  std::vector<api::RoadPositionResult> result(road_position_results);
  result.erase(std::remove_if(result.begin(), result.end(), std::not_fn(filter)), result.end());
  return result;
}

}  // namespace geometry_base
}  // namespace maliput
