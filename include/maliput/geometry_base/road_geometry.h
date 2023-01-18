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
#include <type_traits>
#include <utility>
#include <vector>

#include "maliput/api/basic_id_index.h"
#include "maliput/api/branch_point.h"
#include "maliput/api/junction.h"
#include "maliput/api/road_geometry.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/geometry_base/branch_point.h"
#include "maliput/geometry_base/brute_force_strategy.h"
#include "maliput/geometry_base/junction.h"
#include "maliput/geometry_base/kd_tree_strategy.h"
#include "maliput/geometry_base/strategy_base.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace geometry_base {

/// geometry_base provides basic implementations for a subset of the
/// interfaces of maliput's geometry API (api::RoadGeometry, etc) that
/// can be shared by most "leaf" backends.  It is suitable for use as
/// base classes for a complete backend implementation, or for mock
/// implementations for unit tests (such as
/// test_utilities/mock_geometry.h).
///
/// geometry_base implements all the virtual methods involved in
/// managing the object graph of the road network.  It does not
/// implement any of the fundamental geometric methods that define the
/// immersion of lane-frame into `Inertial`-frame; that is the job of each
/// specific backend.

/// geometry_base's implementation of api::RoadGeometry.
///
/// A base implementation for the DoToRoadPosition and DoToFindRoadPosition virtual methods are offered.
/// These methods could be initialize with two different strategies for performing their tasks:
/// 1. BruteForceStrategy: This strategy performs a brute force search for the nearest lane on the road network.
///                        It performs repetitive queries on the maliput::api::Lane::ToLanePosition method.
/// 2. KdTreeStrategy: This strategy performs a search for the nearest lane on the road network using a KdTree.
///                    In order to achieve this, the kdtree space needs to initialized, which is done by calling
///                    InitializeStrategy() method, after all the lanes have been added to the road geometry.
///
/// The InitializeStrategy() method allows you to indicate which strategy you want to use for the search and it is
/// mandatory to call it before using the DoToRoadPosition and DoToFindRoadPosition methods.
/// By default the BruteForceStrategy is used.
///
/// @code {.cpp}
/// // create road geometry.
/// RoadGeometry road_geometry("road_geometry", 0.1, 0.1, 1.0, math::Vector3(0., 0., 0.));
/// // Add lanes to the road geometry.
/// road_geometry.AddJunction(...);
/// // ...
/// // Add branchpoints to the road geometry.
/// road_geometry.AddBranchPoint(...);
/// // ...
/// // Initialize the strategy.
/// road_geometry->InitializeStrategy<maliput::geometry_base::KDTreeStrategy>(0.25 /* sampling step */);
/// // or road_geometry->InitializeStrategy<maliput::geometry_base::BruteForceStrategy>();
/// @endcode
///
class RoadGeometry : public api::RoadGeometry {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry);

  /// Constructs an empty RoadGeometry with the specified tolerances.
  ///
  /// @param id the ID of the RoadGeometry
  /// @param linear_tolerance the linear tolerance
  /// @param angular_tolerance the angular tolerance
  /// @param scale_length the scale length
  /// @param inertial_to_backend_frame_translation the Inertial Frame to Backend
  ///        Frame translation vector
  ///
  /// @throws maliput::common::assertion_error if either `linear_tolerance` or
  ///         `angular_tolerance` or `scale_length` is non-positive.
  RoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length,
               const math::Vector3& inertial_to_backend_frame_translation)
      : id_(id),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance),
        scale_length_(scale_length),
        inertial_to_backend_frame_translation_(inertial_to_backend_frame_translation) {
    MALIPUT_THROW_UNLESS(linear_tolerance_ > 0.);
    MALIPUT_THROW_UNLESS(angular_tolerance_ > 0.);
    MALIPUT_THROW_UNLESS(scale_length_ > 0.);
  }

  /// Adds @p junction to this RoadGeometry.
  ///
  /// This RoadGeometry will take ownership of `junction` and will be assigned
  /// as its parent.
  ///
  /// @returns `junction`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::Junction.
  ///
  /// @throws std::exception if `junction` is empty.
  template <class T>
  T* AddJunction(std::unique_ptr<T> junction) {
    static_assert(std::is_base_of<Junction, T>::value, "T is not derived from geometry_base::Junction");
    T* const raw_pointer = junction.get();
    AddJunctionPrivate(std::move(junction));
    return raw_pointer;
  }

  /// Adds @p branch_point to this RoadGeometry.
  ///
  /// This RoadGeometry will take ownership of `branch_point` and will be
  /// assigned as its parent.
  ///
  /// @returns `branch_point`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::BranchPoint.
  ///
  /// @throws std::exception if `branch_point` is empty.
  template <class T>
  T* AddBranchPoint(std::unique_ptr<T> branch_point) {
    static_assert(std::is_base_of<BranchPoint, T>::value, "T is not derived from geometry_base::BranchPoint");
    T* const raw_pointer = branch_point.get();
    AddBranchPointPrivate(std::move(branch_point));
    return raw_pointer;
  }

  template <class StrategyT = KDTreeStrategy, class... Args>
  void InitializeStrategy(Args&&... args) {
    static_assert(std::is_base_of<StrategyBase, StrategyT>::value,
                  "StrategyT is not derived from geometry_base::StrategyBase");
    strategy_ = std::make_unique<StrategyT>(this, std::forward<Args>(args)...);
  }

  ~RoadGeometry() override = default;

 private:
  virtual api::RoadPositionResult DoToRoadPosition(
      const api::InertialPosition& inertial_position,
      const std::optional<api::RoadPosition>& hint = std::nullopt) const override;

  virtual std::vector<api::RoadPositionResult> DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                                   double radius) const override;

  // The non-template implementation of AddJunction<T>()
  void AddJunctionPrivate(std::unique_ptr<Junction> junction);

  // The non-template implementation of AddBranchPoint<T>()
  void AddBranchPointPrivate(std::unique_ptr<BranchPoint> branch_point);

  api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return junctions_.size(); }

  const api::Junction* do_junction(int index) const override;

  int do_num_branch_points() const override { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const override;

  const IdIndex& DoById() const override { return id_index_; }

  double do_linear_tolerance() const override { return linear_tolerance_; }

  double do_angular_tolerance() const override { return angular_tolerance_; }

  double do_scale_length() const override { return scale_length_; }

  math::Vector3 do_inertial_to_backend_frame_translation() const override {
    return inertial_to_backend_frame_translation_;
  }

  api::RoadGeometryId id_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  double scale_length_{};
  math::Vector3 inertial_to_backend_frame_translation_{};
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
  api::BasicIdIndex id_index_;
  std::unique_ptr<StrategyBase> strategy_{std::make_unique<BruteForceStrategy>(this)};
};

}  // namespace geometry_base
}  // namespace maliput
