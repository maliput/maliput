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
#include "maliput/base/intersection_book.h"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "maliput/api/rules/phase.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {

using api::Intersection;

class IntersectionBook::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl(const api::RoadGeometry* road_geometry) : road_geometry_(road_geometry) {
    MALIPUT_THROW_UNLESS(road_geometry_ != nullptr);
  }

  ~Impl() = default;

  void AddIntersection(std::unique_ptr<Intersection> intersection) {
    MALIPUT_THROW_UNLESS(intersection != nullptr);
    auto result = book_.emplace(intersection->id(), std::move(intersection));
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple Intersection instances with ID " + intersection->id().string());
    }
  }

  std::vector<Intersection*> DoGetIntersections() const {
    std::vector<Intersection*> intersections;
    auto it = book_.begin();
    while (it != book_.end()) {
      intersections.push_back(it->second.get());
      it++;
    }
    return intersections;
  }

  Intersection* DoGetIntersection(const Intersection::Id& id) const {
    auto it = book_.find(id);
    if (it == book_.end()) {
      return nullptr;
    }
    return it->second.get();
  }

  Intersection* DoGetFindIntersection(const api::InertialPosition& inertial_pos) {
    for (api::Intersection* intersection : DoGetIntersections()) {
      if (intersection->Includes(inertial_pos, road_geometry_)) {
        return intersection;
      }
    }
    return nullptr;
  }

 private:
  const api::RoadGeometry* road_geometry_{};
  std::unordered_map<Intersection::Id, std::unique_ptr<Intersection>> book_;
};

IntersectionBook::IntersectionBook(const api::RoadGeometry* road_geometry)
    : impl_(std::make_unique<Impl>(road_geometry)) {}

IntersectionBook::~IntersectionBook() = default;

void IntersectionBook::AddIntersection(std::unique_ptr<api::Intersection> intersection) {
  impl_->AddIntersection(std::move(intersection));
}

std::vector<Intersection*> IntersectionBook::DoGetIntersections() { return impl_->DoGetIntersections(); }

Intersection* IntersectionBook::DoGetIntersection(const Intersection::Id& id) { return impl_->DoGetIntersection(id); }

std::vector<Intersection*> IntersectionBook::FindIntersections(const std::vector<maliput::api::LaneSRange>& region,
                                                               double tolerance) {
  std::vector<Intersection*> intersections{};
  std::vector<Intersection*> intersections_from_book{GetIntersections()};

  for (const auto& intersection_from_book : intersections_from_book) {
    for (const auto& lane_s_range_intersection : intersection_from_book->region()) {
      const auto lanes_s_range_it = std::find_if(
          region.begin(), region.end(), [lane_s_range_intersection, tolerance](maliput::api::LaneSRange lane_s_range) {
            return lane_s_range_intersection.Intersects(lane_s_range, tolerance);
          });
      if (lanes_s_range_it != region.end()) {
        intersections.push_back(intersection_from_book);
        break;
      }
    }
  }
  return intersections;
}

api::Intersection* IntersectionBook::DoGetFindIntersection(const api::rules::TrafficLight::Id& id) {
  for (const auto& intersection : GetIntersections()) {
    if (intersection->Includes(id)) {
      return intersection;
    }
  }
  return nullptr;
}

api::Intersection* IntersectionBook::DoGetFindIntersection(const api::rules::DiscreteValueRule::Id& id) {
  for (const auto& intersection : GetIntersections()) {
    if (intersection->Includes(id)) {
      return intersection;
    }
  }
  return nullptr;
}

Intersection* IntersectionBook::DoGetFindIntersection(const api::InertialPosition& inertial_pos) {
  return impl_->DoGetFindIntersection(inertial_pos);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
api::Intersection* IntersectionBook::DoGetFindIntersection(const api::rules::RightOfWayRule::Id& id) {
  for (const auto& intersection : GetIntersections()) {
    if (intersection->Includes(id)) {
      return intersection;
    }
  }
  return nullptr;
}
#pragma GCC diagnostic pop

}  // namespace maliput
