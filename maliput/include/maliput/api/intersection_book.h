#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "maliput/api/intersection.h"
#include "maliput/api/rules/traffic_lights.h"

namespace maliput {
namespace api {

/// An abstract interface for providing the mapping from Intersection::Id to
/// Intersection.
class IntersectionBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntersectionBook);

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

 protected:
  IntersectionBook() = default;

 private:
  virtual std::vector<Intersection*> DoGetIntersections() = 0;

  virtual Intersection* DoGetIntersection(const Intersection::Id& id) = 0;

  virtual Intersection* DoGetFindIntersection(const rules::TrafficLight::Id& id) = 0;

  virtual Intersection* DoGetFindIntersection(const rules::DiscreteValueRule::Id& id) = 0;
};

}  // namespace api
}  // namespace maliput
