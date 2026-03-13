// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include <vector>

#include "maliput/api/furniture/road_furniture.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace furniture {

/// Abstract interface for querying RoadFurniture in a road network.
///
/// Follows the NVI pattern consistent with the rest of the maliput API
/// (cf. TrafficLightBook, RoadRulebook).
///
/// The book exposes a unified view (all RoadFurniture items), typed
/// sub-views (RoadSigns, RoadObjects), type-based lookup, and
/// road-location-based lookup.
///
/// This interface supersedes maliput::api::objects::RoadObjectBook. The
/// legacy header in api/objects/ is deprecated.
class RoadFurnitureBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadFurnitureBook)
  virtual ~RoadFurnitureBook() = default;

  /// Returns all RoadFurniture items (signs and objects).
  ///
  /// @returns A vector of pointers to all RoadFurniture items registered in the book.
  std::vector<const RoadFurniture*> RoadFurnitures() const { return DoRoadFurnitures(); }

  /// Returns the RoadFurniture with the given ID, or std::nullopt if not found.
  ///
  /// @param id The unique identifier of the road furniture item.
  /// @returns An optional containing a pointer to the RoadFurniture if found, or std::nullopt if no item with the given ID exists.
  std::optional<const RoadFurniture*> GetRoadFurniture(const RoadFurniture::Id& id) const {
    return DoGetRoadFurniture(id);
  }

  /// Returns all RoadFurniture items of the specified type.
  ///
  /// Both sign types (e.g., RoadFurnitureType::kSpeedLimitSign) and object
  /// types (e.g., RoadFurnitureType::kBarrier) are accepted.
  ///
  /// @param type The RoadFurnitureType to filter by.
  /// @returns A vector of pointers to all RoadFurniture items whose type matches @p type.
  std::vector<const RoadFurniture*> GetByType(RoadFurnitureType type) const { return DoGetByType(type); }

  /// Returns all RoadFurniture items that apply to any lane in @p route.
  ///
  /// "Apply to" is determined by the item's affected_lane_ids() overlapping
  /// with the lane IDs present in @p route.
  ///
  /// @param route The LaneSRoute describing the path of interest.
  /// @returns A vector of pointers to all RoadFurniture items that affect at
  ///          least one lane in @p route.
  std::vector<const RoadFurniture*> GetByLaneSRoute(const LaneSRoute& route) const {
    return DoGetByLaneSRoute(route);
  }

  /// Returns all RoadFurniture items that apply to the specified lane.
  ///
  /// @param lane_id The ID of the lane to query.
  /// @returns A vector of pointers to all RoadFurniture items whose
  ///          affected_lane_ids() includes @p lane_id.
  std::vector<const RoadFurniture*> GetByLane(const LaneId& lane_id) const { return DoGetByLane(lane_id); }

  /// Returns the Lane objects affected by the furniture item with the given ID.
  ///
  /// @param id The unique identifier of the road furniture item.
  /// @returns A vector of pointers to the Lane objects associated with the
  ///          specified furniture item, or an empty vector if the ID is not
  ///          found or no lanes are associated.
  std::vector<const Lane*> GetAffectedLanes(const RoadFurniture::Id& id) const {
    return DoGetAffectedLanes(id);
  }

 protected:
  RoadFurnitureBook() = default;

 private:
  virtual std::vector<const RoadFurniture*> DoRoadFurnitures() const = 0;
  virtual std::optional<const RoadFurniture*> DoGetRoadFurniture(const RoadFurniture::Id& id) const = 0;
  virtual std::vector<const RoadFurniture*> DoGetByType(RoadFurnitureType type) const = 0;
  virtual std::vector<const RoadFurniture*> DoGetByLaneSRoute(const LaneSRoute& route) const = 0;
  virtual std::vector<const RoadFurniture*> DoGetByLane(const LaneId& lane_id) const = 0;
  virtual std::vector<const Lane*> DoGetAffectedLanes(const RoadFurniture::Id& id) const = 0;
};

}  // namespace furniture
}  // namespace api
}  // namespace maliput
