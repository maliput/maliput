// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_network.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/routing/lane_s_range_relation.h"
#include "maliput/routing/phase.h"
#include "maliput/routing/route_position_result.h"

namespace maliput {
namespace routing {

/// Describes the sequence of paths that go from one api::RoadPosition to
/// another.
///
/// It hosts a subset of the api::RoadGeometry that represents a valid routing
/// graph for agent navigation. For scalability, the route is divided into a
/// sequence of Phases. These entities allow agents to reduce the path
/// search space by constraining the lookup towards the end goal.
///
/// Agents are expected to use the Router to obtain a Route. Once in the Route,
/// they can iterate through the Phases or find a specific Phase via
/// an INERTIAL or LANE Frame coordinate and start driving from there towards
/// the end goal.
///
/// The first Phase's first start position identifies the beginning of the
/// Route. The last Phase's first end position identifies the ending of the
/// Route. The sequence of Phases form a continuous route where the end of
/// one Phase exactly matches the beginning of the next Phase in the
/// sequence.
///
/// Let:
/// - `s` indicates the start of the Route.
/// - `e` indicates the end of the Route.
/// - `x` indicates the start or end of an api::LaneSRange.
/// - `*` indicates the start or end of a Phase.
/// - `-` indicates the path of an api::LaneSRange.
/// - `_` indicates the path of an api::LaneSRange within a Phase.
/// - `S0`, `S1`, ...: are api::Segments.
/// - `L0`, `L1`, ...: are api::Lanes.
///
/// Consider the following road geometry and routing request:
///
/// <pre>
///        S1       S2       S3      S4
///                                       e    L0
///                                     /  x   L1
///                                   /  /
///                                 /  /
/// L0 x--------x--------x--------x  /
/// L1 x--------x--------x--------x
/// L2 x--------x--------x--------x--------    L0
///            /
///          /                        S5
///        /
///      /
/// S0 s   L0
/// </pre>
///
///
/// A valid Route could be:
///
/// <pre>
///        S1       S2       S3      S4
///                                       e    L0
///                                     // x   L1
///                                   // //
///                                 // // <-- Phase, index 3
/// L0 x--------x________*________*  //
/// L1 x--------x________*________*
/// L2 x--------*________*________x--------    L0
///            // ^        ^_ Phase, index 2
///          //    \                  S5
///        //        Phase, index 1
///      //   <-- Phase, index 0
/// S0 s   L0
/// </pre>
///
/// Depending on where queried, different api::LaneSRoutes are returned, for
/// example, if queried from `s`, the returned sequence will be:
/// {S0:L0, S2:L2, S2:L1, S2:L0, S3:L0, S3:L1, S3:L2, S4:L0, S4:L1}
///
/// And if queried at any point in S3:L2: {S3:L2, S3:L1, S3:L0, S4:L0, S4:L1}.
class Route final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Route);
  Route() = delete;

  /// Constructs a Route.
  ///
  /// @param phases The sequence of Phases. It must not be empty.
  /// Phases must be connected end to end.
  /// @param road_network The api::RoadNetwork pointer. It must not be
  /// nullptr. The lifetime of this pointer must exceed that of this object.
  /// @throws common::assertion_error When @p phases is empty.
  /// @throws common::assertion_error When @p phases is not connected end
  /// to end.
  /// @throws common::assertion_error When @p road_network is nullptr.
  Route(const std::vector<Phase>& phases, const api::RoadNetwork* road_network);

  /// @return The number of Phases.
  int size() const { return static_cast<int>(phases_.size()); }

  /// Indexes the Phases.
  ///
  /// @param index The index of the Phase. It must be non-negative and
  /// less than `size()`.
  /// @return The Phase at @p index.
  /// @throws std::out_of_range When @p index is negative or >= `size()`.
  const Phase& Get(int index) const { return phases_.at(index); }

  /// Indexes an api::LaneSRange in a Phase.
  ///
  /// Convenient method for advanced users.
  ///
  /// @param phase_index The index of the api::LaneSRange in the Phase specified by @p phase_index. It must be
  /// non-negative and less than `size()`.
  /// @param lane_s_range_index The index of the api::LaneSRange. It must be
  /// non-negative and less than `Phase::lane_s_ranges().size()`.
  /// @return The api::LaneSRange indexed at the @p phase_index -th
  /// Phase at the @p lane_s_range_index -th position.
  /// @throws std::out_of_range When any of the preconditions of
  /// @p phase_index or @p lane_s_range_index are unmet.
  const api::LaneSRange& GetLaneSRange(int phase_index, int lane_s_range_index) const {
    return Get(phase_index).lane_s_ranges().at(lane_s_range_index);
  }

  /// Returns the start of this Route. This is a convenience method for
  /// Get(0).start_positions().front().
  const api::RoadPosition& start_route_position() const { return phases_.front().start_positions().front(); }

  /// Returns the end of this Route. This is a convenience method for
  /// Get(size() - 1).end_positions().front().
  const api::RoadPosition& end_route_position() const { return phases_.back().end_positions().front(); }

  /// Finds the RoutePositionResult which @p inertial_position best fits.
  ///
  /// The fitting of the @p inertial_position into the complete Route will use
  /// the same set of rules api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching api::RoadPositionResult within the api::RoadGeometry.
  /// When the @p inertial_position does not fall into the volume defined by the
  /// set of api::LaneSRanges each Phase has, the returned
  /// Phase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame are evaluated there as well.
  ///
  /// @param inertial_position The INERTIAL-Frame position.
  /// @return A RoutePositionResult.
  RoutePositionResult FindRoutePosition(const api::InertialPosition& inertial_position) const;

  /// Finds the RoutePositionResult which @p road_position best fits.
  ///
  /// The fitting of the @p road_position into the complete Route will use
  /// the same set of rules api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching api::RoadPositionResult within the api::RoadGeometry.
  /// When the @p road_position does not fall into the volume defined by the
  /// set of api::LaneSRanges each Phase has, the returned
  /// Phase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame are evaluated there as well.
  ///
  /// @param road_position The road position. It must be valid.
  /// @return A RoutePositionResult.
  /// @throws common::assertion_error When @p road_position is not
  /// valid.
  RoutePositionResult FindRoutePosition(const api::RoadPosition& road_position) const;

  /// Computes the relation between @p lane_s_range_b with respect to
  /// @p lane_s_range_a.
  ///
  /// @param lane_s_range_a An api::LaneSRange.
  /// @param lane_s_range_b An api::LaneSRange.
  /// @return The LaneSRangeRelation between @p lane_s_range_b with respect to
  /// @p lane_s_range_a.
  LaneSRangeRelation ComputeLaneSRangeRelation(const api::LaneSRange& lane_s_range_a,
                                               const api::LaneSRange& lane_s_range_b) const;

  /// Computes an api::LaneSRoute that connects @p start_position with
  /// end_route_position().
  ///
  /// The resulting api::LaneSRoute aims for reducing the number of lane
  /// switches.
  /// When @p start_position is not within this Route, it will be transformed by
  /// FindRoutePositionBy() to a point within it and then the api::LaneSRoute
  /// will be computed.
  ///
  /// Simple agents / users should consider using this method to avoid dealing with
  /// the complexities of api::LaneSRange switching within a Phase and between Phases.
  ///
  /// @param start_position The start api::RoadPosition of this path. It must be
  /// valid.
  /// @return The api::LaneSRoute connecting @p start_position and
  /// end_route_position().
  /// @throws common::assertion_error When @p start_position is not valid.
  api::LaneSRoute ComputeLaneSRoute(const api::RoadPosition& start_position) const;

  // TODO(maliput#628): Implement me.
  std::vector<std::string> ValidateEndToEndConnectivity() const { return {}; }

 private:
  // @{

  // The direction of travel along the Routes is defined by the underlying Phases' api::LaneSRanges.
  // Each api::LaneSRange by means of its api::SRange::WithS() defines the direction of travel.
  // When true, forward means in +s direction, and left means in the +r direction. Otherwise,
  // forward means in the -s direction, and left means in the -r direction.

  // Defines the sign and increment of one unit towards the right of the index of
  // api::LaneSRanges in a Phase.
  static constexpr int kTowardsRight{-1};

  // Defines the sign and increment of one unit towards the left of the index of
  // api::LaneSRanges in a Phase.
  static constexpr int kTowardsLeft{1};

  // @}

  // Convenient type to index an api::LaneSRange within this Route.
  struct LaneSRangeIndex {
    // @brief Index the Phase within this Route.
    size_t phase{0u};
    // @brief Index the api::LaneSRange within `phase`-th Phase.
    size_t lane_s_range{0u};
  };

  // Finds the LaneSRangeIndex for an api::LaneSRange.
  //
  // To provide a match, @p lane_s_range must be within RoadNetwork's tolerance
  // of any of the existing api::LaneSRanges within this Route.
  //
  // @param lane_s_range The api::LaneSRange to look for.
  // @return An optional containing the index of the api::LaneSRange within this
  // Route.
  std::optional<LaneSRangeIndex> FindLaneSRangeIndex(const api::LaneSRange& lane_s_range) const;

  // Finds the LaneSRangeIndex of the api::LaneSRange that is
  // LaneSRelation::kPreceedingStraight with respect to @p index api::LaneSRange.
  //
  // @param index The index of the api::LaneSRange to find its
  // LaneSRelation::kPreceedingStraight counterpart. It must be a valid index.
  // @return An optional containing the LaneSRangeIndex of the api::LaneSRange.
  std::optional<LaneSRangeIndex> FindStraightPredecessor(const LaneSRangeIndex& index) const;

  // Finds how to move the index of api::LaneSRanges within a Phase to find
  // the predecessor of the api::LanesRange at @p index.
  //
  // @param index The index of api::LaneSRange within this Route. The phase index must not be zero.
  // @return kTowardsLeft When moving towards the left within the Phase,
  // otherwise kTowardsRight.
  // @throws common::assertion_error When `index.first` is zero as there is no predecessor.
  int FindDirectionTowardsLaneSRangeWithStraightPredecessor(const LaneSRangeIndex& index) const;

  std::vector<Phase> phases_;
  std::unordered_map<api::LaneId, std::vector<LaneSRangeIndex>> lane_id_to_indices_;
  const api::RoadNetwork* road_network_{};
};

}  // namespace routing
}  // namespace maliput
