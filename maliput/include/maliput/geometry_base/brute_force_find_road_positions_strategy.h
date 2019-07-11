#pragma once

#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"

namespace maliput {
namespace geometry_base {

/// Provides a brute force implementation of RoadGeometry::FindRoadPosition()
/// that exhaustively calls `ToLanePosition()` on all lanes and checks
/// _distance_ with `radius`.
///
/// Maliput backends could avoid implementing a custom implementation that knows
/// about the geometry internals by forwarding calls to this function.
///
/// @param rg The RoadGeometry over all these operations are performed. It
///        must not be nullptr.
/// @param geo_position The geo position to convert into one or more
///        RoadPositions.
/// @param radius The maximum distance from @p geo_position to search. It
///        must not be negative.
/// @return A vector of RoadPositionResults representing the possible
///         RoadPositions.
/// @throws std::runtime_error If rg is nullptr, or any entity within it is
///         nullptr.
/// @throws std::runtime_error If radius is negative.
std::vector<maliput::api::RoadPositionResult>
BruteForceFindRoadPositionsStrategy(
    const maliput::api::RoadGeometry* rg,
    const maliput::api::GeoPosition& geo_position,
    double radius);

}  // namespace geometry_base
}  // namespace maliput
