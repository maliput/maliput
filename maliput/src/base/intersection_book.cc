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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() = default;
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

 private:
  std::unordered_map<Intersection::Id, std::unique_ptr<Intersection>> book_;
};

IntersectionBook::IntersectionBook() : impl_(std::make_unique<Impl>()) {}

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
    return intersection->Includes(id) ? intersection : nullptr;
  }
  return nullptr;
}

api::Intersection* IntersectionBook::DoGetFindIntersection(const api::rules::DiscreteValueRule::Id& id) {
  for (const auto& intersection : GetIntersections()) {
    return intersection->Includes(id) ? intersection : nullptr;
  }
  return nullptr;
}

}  // namespace maliput
