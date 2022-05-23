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

#include <memory>

#include "maliput/api/intersection.h"
#include "maliput/api/intersection_book.h"
#include "maliput/api/road_geometry.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {

/// A concrete implementation of the api::IntersectionBook abstract interface.
class IntersectionBook : public api::IntersectionBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(IntersectionBook);

  /// Constructs an IntersectionBook
  ///
  /// @param road_geometry The api::RoadGeometry used when computing GetFindIntersection(inertial_pose)
  /// @pre road_geometry must not be null.
  /// @throws maliput::common::assertion_error when preconditions are unmet.
  explicit IntersectionBook(const api::RoadGeometry* road_geometry);

  ~IntersectionBook() override;

  /// Adds @p intersection to this IntersectionBook.
  ///
  /// @throws std::exception if an api::Intersection with the same ID already
  /// exists, or if @p intersection is nullptr.
  void AddIntersection(std::unique_ptr<api::Intersection> intersection);

  /// Returns a vector of Intersection pointers whose regions intersect `region`.
  ///
  /// @param region A vector of api::LaneSRanges.
  /// @param tolerance Tolerance to compare api::LaneSRange intersections.
  std::vector<api::Intersection*> FindIntersections(const std::vector<api::LaneSRange>& region, double tolerance);

 private:
  std::vector<api::Intersection*> DoGetIntersections() override;

  api::Intersection* DoGetIntersection(const api::Intersection::Id& id) override;

  api::Intersection* DoGetFindIntersection(const api::rules::TrafficLight::Id& id) override;

  api::Intersection* DoGetFindIntersection(const api::rules::DiscreteValueRule::Id& id) override;

  api::Intersection* DoGetFindIntersection(const api::InertialPosition& inertial_pos) override;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  api::Intersection* DoGetFindIntersection(const api::rules::RightOfWayRule::Id& id) override;
#pragma GCC diagnostic pop

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
