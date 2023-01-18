// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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

#include <memory>
#include <optional>
#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"

namespace maliput {
namespace geometry_base {

/// Compares two maliput::api::RoadPositionResults and determines whether @p new_road_position_result
/// is closer to @p road_position_result.
/// @see maliput::api::RoadPosition::ToRoadPosition() for a clear definition of how the comparison must be performed.
/// @param  new_road_position_result A maliput::api::RoadPositionResult to compare.
/// @param  road_position_result The reference maliput::api::RoadPositionResult to compare.
/// @return true When @p new_road_position_result is closer than @p road_position_result.
bool IsNewRoadPositionResultCloser(const maliput::api::RoadPositionResult& new_road_position_result,
                                   const maliput::api::RoadPositionResult& road_position_result);

/// Provides a base interface for defining strategies that will affect the behavior
/// of the queries RoadGeomoetry::ToRoadPosition and RoadGeomoetry::FindRoadPositions.
/// See RoadGeometry::InitializeStrategy().
class StrategyBase {
 public:
  virtual ~StrategyBase() = default;

  api::RoadPositionResult ToRoadPosition(const api::InertialPosition& inertial_position,
                                         const std::optional<api::RoadPosition>& hint) const {
    return DoToRoadPosition(inertial_position, hint);
  }

  std::vector<api::RoadPositionResult> FindRoadPositions(const api::InertialPosition& inertial_position,
                                                         double radius) const {
    return DoFindRoadPositions(inertial_position, radius);
  }

 protected:
  StrategyBase(const api::RoadGeometry* rg) : rg_(rg) { MALIPUT_THROW_UNLESS(rg_ != nullptr); }

  const api::RoadGeometry* get_road_geometry() const { return rg_; }

 private:
  virtual api::RoadPositionResult DoToRoadPosition(const api::InertialPosition& inertial_position,
                                                   const std::optional<api::RoadPosition>& hint) const = 0;

  virtual std::vector<api::RoadPositionResult> DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                                   double radius) const = 0;

  const api::RoadGeometry* rg_{};
};

}  // namespace geometry_base
}  // namespace maliput
