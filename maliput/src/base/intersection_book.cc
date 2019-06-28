#include "maliput/base/intersection_book.h"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_throw.h"

namespace maliput {

using api::Intersection;

class IntersectionBook::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() = default;
  ~Impl() = default;

  void AddIntersection(std::unique_ptr<Intersection> intersection) {
    DRAKE_THROW_UNLESS(intersection != nullptr);
    auto result = book_.emplace(intersection->id(), std::move(intersection));
    if (!result.second) {
      throw std::logic_error(
          "Attempted to add multiple Intersection instances with ID " +
          intersection->id().string());
    }
  }

  std::vector<Intersection*> DoGetIntersections() const {
    std::vector<Intersection*> intersections;
    auto it = book_.begin();
    while(it != book_.end()) {
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

void IntersectionBook::AddIntersection(
    std::unique_ptr<api::Intersection> intersection) {
  impl_->AddIntersection(std::move(intersection));
}

std::vector<Intersection*> IntersectionBook::DoGetIntersections() {
  return impl_->DoGetIntersections();
}

Intersection* IntersectionBook::DoGetIntersection(const Intersection::Id& id) {
  return impl_->DoGetIntersection(id);
}

}  // namespace maliput
