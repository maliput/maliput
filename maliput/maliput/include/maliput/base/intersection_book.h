#pragma once

#include <memory>

#include "maliput/api/intersection.h"
#include "maliput/api/intersection_book.h"
#include "drake/common/drake_copyable.h"

namespace maliput {

/// A concrete implementation of the api::IntersectionBook abstract interface.
class IntersectionBook : public api::IntersectionBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntersectionBook);

  IntersectionBook();

  ~IntersectionBook() override;

  /// Adds @p intersection to this IntersectionBook.
  ///
  /// @throws std::exception if an api::Intersection with the same ID already
  /// exists, or if @p intersection is nullptr.
  void AddIntersection(std::unique_ptr<api::Intersection> intersection);

 private:
  std::vector<api::Intersection*> DoGetIntersections() override;

  api::Intersection* DoGetIntersection(const api::Intersection::Id& id)
      override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
