#pragma once

#include "maliput/api/road_network.h"

namespace maliput {
namespace api {

/// Provides configuration options for the RoadNetworkValidator.
struct RoadNetworkValidatorOptions {
  /// Whether to check if the DirectionUsageRules within a RoadNetwork cover
  /// the RoadNetwork's RoadGeometry.
  bool check_direction_usage_rule_coverage{true};
  /// Whether to check the RoadGeometry invariants.
  /// Please, @see RoadGeometry::CheckInvariants() for further details.
  bool check_road_geometry_invariants{true};
  /// Whether to check if the RoadGeometry hierarchy is correct.
  /// Hierarchy checks include:
  /// - At least one Junction in the RoadGeometry.
  /// - At least one Segment in each Junction.
  /// - At least one Lane in each Segment.
  /// - At least two BranchPoints in the RoadGeometry.
  /// - Parent-children relations between RoadGeometry and Junctions, Junction
  ///   and Segments, Segment and Lanes, and RoadGeometry and BranchPoints are
  ///   correctly set.
  /// - Every Lane is attached to a BranchPoint at each end.
  /// - Every BranchPoint's A-Side and B-Side are not nullptr.
  /// - Every BranchPoint is not empty, i.e. either A-Side or B-Side have at
  ///   least one Lane.
  bool check_road_geometry_hierarchy{true};
  /// Whether to check that RelatedBulbGroups in RightOfWayRules have
  /// supporting TrafficLight objects in TrafficLightBook and BulbGroups are
  /// within the right TrafficLights.
  bool check_related_bulb_groups{true};
};

/// Validates a RoadNetwork.
///
/// @param road_network The RoadNetwork to validate.
/// @param options Options for selecting what aspects of RoadNetwork to check.
/// @throws maliput::common::assertion_error When @p road_network is not valid.
void ValidateRoadNetwork(const RoadNetwork& road_network, const RoadNetworkValidatorOptions& options);

}  // namespace api
}  // namespace maliput
