// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
  /// @param inertial_pos A position in Inertial-Frame.
  /// @returns The Intersection that contains `inertial_pose`. When none of the Intersections have overlap with
  /// `inertial_pose`, nullptr is returned.
  Intersection* FindIntersection(const InertialPosition& inertial_pos) { return DoGetFindIntersection(inertial_pos); }

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

  virtual Intersection* DoGetFindIntersection(const InertialPosition& inertial_pos) = 0;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  virtual Intersection* DoGetFindIntersection(const rules::RightOfWayRule::Id& id) = 0;
#pragma GCC diagnostic pop
};

}  // namespace api
}  // namespace maliput
