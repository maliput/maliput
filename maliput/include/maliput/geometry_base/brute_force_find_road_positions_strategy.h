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
/// @note Maliput backends could avoid implementing a custom implementation that
/// knows about the geometry internals by forwarding calls to this function.
///
/// @param rg The RoadGeometry over all these operations are performed. It
///        must not be nullptr.
/// @param inertial_position The inertial position to convert into one or more
///        RoadPositions.
/// @param radius The maximum distance from @p inertial_position to search. It
///        must not be negative.
/// @return A vector of RoadPositionResults representing the possible
///         RoadPositions.
/// @throws maliput::common::assertion_error If rg is nullptr, or any entity
///         within it is nullptr.
/// @throws maliput::common::assertion_error If radius is negative.
std::vector<maliput::api::RoadPositionResult> BruteForceFindRoadPositionsStrategy(
    const maliput::api::RoadGeometry* rg, const maliput::api::InertialPosition& inertial_position, double radius);

}  // namespace geometry_base
}  // namespace maliput
