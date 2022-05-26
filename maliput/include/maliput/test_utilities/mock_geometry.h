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

#include <optional>

#include "maliput/api/lane_data.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/geometry_base/branch_point.h"
#include "maliput/geometry_base/junction.h"
#include "maliput/geometry_base/lane.h"
#include "maliput/geometry_base/road_geometry.h"
#include "maliput/geometry_base/segment.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace geometry_base {
namespace test {

/// @file
/// Mock concrete implementation of maliput geometry API, useful for tests.
///
/// The classes in this file are concrete implementations of the
/// maliput geometry API, built on top of the @ref
/// maliput::geometry_base "geometry_base" base classes.  The
/// only difference between these classes and @ref
/// maliput::geometry_base "geometry_base" is that all the
/// remaining pure virtual methods in @ref
/// maliput::geometry_base "geometry_base" (i.e., the methods
/// involving actual lane-frame and `Inertial`-frame geometry) have been
/// provided with implementations that simply throw `std::exception`.
/// These "Mock" classes do provide sufficient functionality to
/// exercise the geometry API's object graph.
///
/// All virtual methods are overridable (i.e., none are marked `final`).
/// Test implementors may re-implement methods as they see fit.

/// Mock api::RoadGeometry implementation; see mock_geometry.h.
class MockRoadGeometry : public geometry_base::RoadGeometry {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadGeometry);

  /// Constructs an empty MockRoadGeometry with the specified tolerances.
  ///
  /// @param id the ID of the RoadGeometry
  /// @param linear_tolerance the linear tolerance
  /// @param angular_tolerance the angular tolerance
  /// @param scale_length the scale length
  /// @param inertial_to_backend_frame_translation the Inertial to Backend Frame
  ///        translation
  ///
  /// @throws std::exception if either `linear_tolerance` or
  ///         `angular_tolerance` or `scale_length` is non-positive.
  MockRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance,
                   double scale_length, const math::Vector3& inertial_to_backend_frame_translation)
      : geometry_base::RoadGeometry(id, linear_tolerance, angular_tolerance, scale_length,
                                    inertial_to_backend_frame_translation) {}

 private:
  api::RoadPositionResult DoToRoadPosition(const api::InertialPosition& inertial_position,
                                           const std::optional<api::RoadPosition>& hint) const override;

  std::vector<api::RoadPositionResult> DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                           double radius) const override;
};

/// Mock api::BranchPoint implementation; see mock_geometry.h.
class MockBranchPoint : public geometry_base::BranchPoint {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockBranchPoint);

  /// Constructs a partially-initialized MockBranchPoint.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::BranchPoint for discussion on initialization.
  explicit MockBranchPoint(const api::BranchPointId& id) : geometry_base::BranchPoint(id) {}
};

/// Mock api::Junction implementation; see mock_geometry.h.
class MockJunction : public geometry_base::Junction {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockJunction);

  /// Constructs a partially-initialized MockJunction.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::Junction for discussion on initialization.
  explicit MockJunction(const api::JunctionId& id) : geometry_base::Junction(id) {}
};

/// Mock api::Segment implementation; see mock_geometry.h.
class MockSegment : public geometry_base::Segment {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockSegment);

  /// Constructs a partially-initialized MockSegment.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::Segment for discussion on initialization.
  explicit MockSegment(const api::SegmentId& id) : geometry_base::Segment(id) {}
};

/// Mock api::Lane implementation; see mock_geometry.h.
class MockLane : public geometry_base::Lane {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockLane);

  /// Constructs a partially-initialized MockLane.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::Lane for discussion on initialization.
  explicit MockLane(const api::LaneId& id) : geometry_base::Lane(id) {}

 private:
  double do_length() const override;
  api::RBounds do_lane_bounds(double) const override;
  api::RBounds do_segment_bounds(double) const override;
  api::HBounds do_elevation_bounds(double, double) const override;
  api::InertialPosition DoToInertialPosition(const api::LanePosition& lane_pos) const override;
  api::Rotation DoGetOrientation(const api::LanePosition& lane_pos) const override;
  api::LanePosition DoEvalMotionDerivatives(const api::LanePosition& position,
                                            const api::IsoLaneVelocity& velocity) const override;
  api::LanePositionResult DoToLanePosition(const api::InertialPosition& inertial_position) const override;
};

}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
