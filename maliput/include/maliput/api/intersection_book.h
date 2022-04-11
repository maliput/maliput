#pragma once

#include <vector>

#include "maliput/api/intersection.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {
namespace api {

/// An abstract interface for providing the mapping from Intersection::Id to
/// Intersection.
class IntersectionBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(IntersectionBook);

  virtual ~IntersectionBook() = default;

  /// Gets a list of all Intersections within this book.
  std::vector<Intersection*> GetIntersections() { return DoGetIntersections(); }

  /// Gets the specified Intersection. Returns nullptr if @p id is unrecognized.
  /// Otherwise, the returned pointer is guaranteed to remain valid throughout
  /// the lifetime of this IntersectionBook's instance.
  Intersection* GetIntersection(const Intersection::Id& id) { return DoGetIntersection(id); }

  /// Find the intersection which contains api::rules::TrafficLight::Id.
  ///
  /// @param id A rules::TrafficLight::Id.
  /// @returns The Intersection that contains `id`. When none of the Intersections have a rules::TrafficLight with `id`,
  /// nullptr is returned.
  Intersection* FindIntersection(const rules::TrafficLight::Id& id) { return DoGetFindIntersection(id); }

  /// Find the intersection which contains api::rules::DiscreteValueRule::Id.
  ///
  /// @param id A rules::DiscreteValueRule::Id.
  /// @returns The Intersection that contains `id`. When none of the Intersections have a rules::DiscreteValueRule with
  /// `id`, nullptr is returned.
  Intersection* FindIntersection(const rules::DiscreteValueRule::Id& id) { return DoGetFindIntersection(id); }

  /// Find the intersection which contains api::rules::DiscreteValueRule::Id.
  ///
  /// @param inertial_pose A position in inertial space (TODO TERMINOLOGY space?)
  /// @returns The Intersection that contains `inertial_pose`. When none of the Intersections have overlap with
  /// `inertial_pose`, nullptr is returned.
  Intersection* FindIntersection(const InertialPosition& inertial_pose, const api::RoadGeometry* road_geometry) { return DoGetFindIntersection(inertial_pose, road_geometry); }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Find the intersection which contains api::rules::RightOfWayRule::Id.
  ///
  /// @param id A rules::RightOfWayRule::Id.
  /// @returns The Intersection that contains `id`. When none of the Intersections have a rules::RightOfWayRule with
  /// `id`, nullptr is returned.
  MALIPUT_DEPRECATED("RightOfWayRule class will be deprecated")
  Intersection* FindIntersection(const rules::RightOfWayRule::Id& id) { return DoGetFindIntersection(id); }
#pragma GCC diagnostic pop

 protected:
  IntersectionBook() = default;

 private:
  virtual std::vector<Intersection*> DoGetIntersections() = 0;

  virtual Intersection* DoGetIntersection(const Intersection::Id& id) = 0;

  virtual Intersection* DoGetFindIntersection(const rules::TrafficLight::Id& id) = 0;

  virtual Intersection* DoGetFindIntersection(const rules::DiscreteValueRule::Id& id) = 0;

  virtual Intersection* DoGetFindIntersection(const InertialPosition& inertial_pose, const api::RoadGeometry* road_geometry) = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  virtual Intersection* DoGetFindIntersection(const rules::RightOfWayRule::Id& id) = 0;
#pragma GCC diagnostic pop
};

}  // namespace api
}  // namespace maliput
