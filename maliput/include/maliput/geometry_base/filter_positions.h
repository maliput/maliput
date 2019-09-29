#pragma once

#include <vector>

#include "maliput/api/lane_data.h"

namespace maliput {
namespace geometry_base {

/// @returns A filtered version `road_position_results` by applying `filter`
/// to each item of `road_position_results`. When `filter` returns true, the
/// item is preserved.
///
/// @param road_position_results A vector of RoadPositionResults to be
/// filtered. Typically, it is the result of calling
/// api::RoadGeometry::FindRoadPositions().
/// @param filter An arbitrary functor to filter `road_position_results`.
std::vector<api::RoadPositionResult> FilterRoadPositionResults(
    const std::vector<api::RoadPositionResult>& road_position_results,
    const std::function<bool(const api::RoadPositionResult&)>& filter);

}  // namespace geometry_base
}  // namespace maliput
