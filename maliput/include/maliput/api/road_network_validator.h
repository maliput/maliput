#pragma once

#include "maliput/api/road_network.h"

namespace maliput {
namespace api {

/// Provides configuration options for the RoadNetworkValidator.
struct RoadNetworkValidatorOptions {
  /// Whether to check if the DirectionUsageRules within a RoadNetwork cover
  /// the RoadNetwork's RoadGeometry.
  bool check_direction_usage_rule_coverage{true};
  /// Whether to check if the RoadGeometry within a RoadNetwork satisfies G1
  /// constraint at BranchPoints.
  bool check_road_geometry_invariants{true};
  /// Whether to check if the RoadGeometry hierarchy is correct.
  bool check_road_geometry_hierarchy{true};
};

/// Validates a RoadNetwork.
///
/// @param road_network The RoadNetwork to validate.
/// @param options Options for selecting what aspects of RoadNetwork to check.
/// @throws maliput::common::assertion_error When @p road_network is not valid.
void ValidateRoadNetwork(const RoadNetwork& road_network, const RoadNetworkValidatorOptions& options);

}  // namespace api
}  // namespace maliput
