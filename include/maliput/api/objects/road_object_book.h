// BSD 3-Clause License
//
// Copyright (c) 2026, Woven Planet. All rights reserved.
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

#include <string>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/objects/road_object.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace objects {

/// Abstract interface for providing access to RoadObjects in a road network.
///
/// RoadObjectBook provides methods to query road objects by various criteria
/// such as ID, type, position, or associated road. Implementations are provided
/// by specific backends (e.g., maliput_malidrive for OpenDRIVE maps).
class RoadObjectBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadObjectBook);

  virtual ~RoadObjectBook() = default;

  /// Returns all RoadObjects in this book.
  std::vector<const RoadObject*> RoadObjects() const { return DoRoadObjects(); }

  /// Gets the RoadObject with the specified ID.
  ///
  /// @param id The unique identifier of the road object.
  /// @returns A pointer to the RoadObject, or nullptr if not found.
  const RoadObject* GetRoadObject(const RoadObject::Id& id) const { return DoGetRoadObject(id); }

  /// Returns all RoadObjects of the specified type.
  ///
  /// @param type The type of road objects to retrieve.
  /// @returns A vector of pointers to matching RoadObjects.
  std::vector<const RoadObject*> GetRoadObjectsByType(RoadObjectType type) const {
    return DoGetRoadObjectsByType(type);
  }

  /// Returns all RoadObjects associated with the specified lane.
  ///
  /// @param lane_id The identifier of the lane.
  /// @returns A vector of pointers to RoadObjects associated with that lane.
  std::vector<const RoadObject*> GetRoadObjectsByLane(const LaneId& lane_id) const {
    return DoGetRoadObjectsByLane(lane_id);
  }

  /// Returns all RoadObjects within a specified radius of an inertial position.
  ///
  /// @param position The center position for the search.
  /// @param radius The search radius in meters.
  /// @returns A vector of pointers to RoadObjects within the specified radius.
  std::vector<const RoadObject*> FindRoadObjectsInRadius(const InertialPosition& position, double radius) const {
    return DoFindRoadObjectsInRadius(position, radius);
  }

 protected:
  RoadObjectBook() = default;

 private:
  /// @name NVI (Non-Virtual Interface) methods
  /// @{
  virtual std::vector<const RoadObject*> DoRoadObjects() const = 0;

  virtual const RoadObject* DoGetRoadObject(const RoadObject::Id& id) const = 0;

  virtual std::vector<const RoadObject*> DoGetRoadObjectsByType(RoadObjectType type) const = 0;

  virtual std::vector<const RoadObject*> DoGetRoadObjectsByLane(const LaneId& lane_id) const = 0;

  virtual std::vector<const RoadObject*> DoFindRoadObjectsInRadius(const InertialPosition& position,
                                                                   double radius) const = 0;
  /// @}
};

}  // namespace objects
}  // namespace api
}  // namespace maliput
