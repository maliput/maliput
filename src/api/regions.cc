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
#include "maliput/api/regions.h"

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"

namespace maliput {
namespace api {

namespace {

// Evaluates whether `lane_s_range` belongs to `road_geometry`.
bool IsValid(const LaneSRange& lane_s_range, const RoadGeometry* road_geometry) {
  // TODO(francocipollone) This function could be removed once maliput#203 is addressed.
  return road_geometry->ById().GetLane(lane_s_range.lane_id()) != nullptr;
}

}  // namespace

bool IsContiguous(const LaneSRange& lane_range_a, const LaneSRange& lane_range_b, const RoadGeometry* road_geometry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  const Lane* lane_a = road_geometry->ById().GetLane(lane_range_a.lane_id());
  MALIPUT_THROW_UNLESS(lane_a != nullptr);
  const Lane* lane_b = road_geometry->ById().GetLane(lane_range_b.lane_id());
  MALIPUT_THROW_UNLESS(lane_b != nullptr);
  // Evaluates G1 contiguity at the end and start of `lane_a` and `lane_b` respectively.
  const InertialPosition inertial_position_a =
      lane_a->ToInertialPosition(LanePosition(lane_range_a.s_range().s1(), 0, 0));
  const InertialPosition inertial_position_b =
      lane_b->ToInertialPosition(LanePosition(lane_range_b.s_range().s0(), 0, 0));
  const Rotation lane_a_rot = lane_a->GetOrientation(LanePosition(lane_range_a.s_range().s1(), 0, 0));
  const Rotation lane_b_rot = lane_b->GetOrientation(LanePosition(lane_range_b.s_range().s0(), 0, 0));
  return inertial_position_a.Distance(inertial_position_b) < road_geometry->linear_tolerance() &&
         lane_a_rot.Distance(lane_b_rot) < road_geometry->angular_tolerance();
}

SRange::SRange(double s0, double s1) : s0_(s0), s1_(s1) {
  MALIPUT_THROW_UNLESS(s0_ >= 0);
  MALIPUT_THROW_UNLESS(s1_ >= 0);
}

void SRange::set_s0(double s0) {
  MALIPUT_THROW_UNLESS(s0 >= 0);
  s0_ = s0;
}

void SRange::set_s1(double s1) {
  MALIPUT_THROW_UNLESS(s1 >= 0);
  s1_ = s1;
}

bool SRange::Intersects(const SRange& s_range, double tolerance) const {
  MALIPUT_THROW_UNLESS(std::min(s0(), s1()) >= 0 && std::min(s_range.s0(), s_range.s1()) >= 0);
  if (tolerance < 0.) {
    // When it is negative, tolerance's absolute value can not be bigger than half size of minor SRange.
    MALIPUT_THROW_UNLESS(std::min(size(), s_range.size()) / 2. >= std::fabs(tolerance));
  }
  const SRange wider_s_range(std::max(std::min(s0(), s1()) - tolerance, 0.), std::max(s0(), s1()) + tolerance);
  return !((std::max(s_range.s0(), s_range.s1()) < wider_s_range.s0()) ||
           (std::min(s_range.s0(), s_range.s1()) > wider_s_range.s1()));
}

std::optional<SRange> SRange::GetIntersection(const SRange& s_range, double tolerance) const {
  if (Intersects(s_range, tolerance)) {
    const SRange wider_s_range(std::max(std::min(s0(), s1()) - tolerance, 0.), std::max(s0(), s1()) + tolerance);
    const double max = std::max(s_range.s0(), s_range.s1()) >= wider_s_range.s1()
                           ? std::max(s0(), s1())
                           : std::max(s_range.s0(), s_range.s1());
    const double min = std::min(s_range.s0(), s_range.s1()) <= wider_s_range.s0()
                           ? std::min(s0(), s1())
                           : std::min(s_range.s0(), s_range.s1());

    return std::optional<SRange>{SRange(min, max)};
  }
  return std::nullopt;
}

bool LaneSRange::Intersects(const LaneSRange& lane_s_range, const double tolerance) const {
  return lane_id_ == lane_s_range.lane_id() ? s_range_.Intersects(lane_s_range.s_range(), tolerance) : false;
}

std::optional<LaneSRange> LaneSRange::GetIntersection(const LaneSRange& lane_s_range, double tolerance) const {
  if (Intersects(lane_s_range, tolerance)) {
    const auto intersection = s_range_.GetIntersection(lane_s_range.s_range(), tolerance);
    MALIPUT_THROW_UNLESS(intersection.has_value());
    return std::optional<LaneSRange>{LaneSRange(lane_id_, intersection.value())};
  }
  return std::nullopt;
}

bool LaneSRoute::Intersects(const LaneSRoute& lane_s_route, double tolerance) const {
  for (const auto& s_range : ranges()) {
    const auto lane_s_range_it =
        std::find_if(lane_s_route.ranges().begin(), lane_s_route.ranges().end(),
                     [s_range](const LaneSRange& lane_s_range) { return s_range.lane_id() == lane_s_range.lane_id(); });
    if (lane_s_range_it != lane_s_route.ranges().end()) {
      if (s_range.Intersects(*lane_s_range_it, tolerance)) {
        return true;
      }
    }
  }
  return false;
}

bool IsIncluded(const InertialPosition& inertial_position, const std::vector<LaneSRange>& lane_s_ranges,
                const RoadGeometry* road_geometry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(!lane_s_ranges.empty());
  for (const auto& lane_s_range : lane_s_ranges) {
    MALIPUT_THROW_UNLESS(IsValid(lane_s_range, road_geometry));
  }
  const double linear_tolerance = road_geometry->linear_tolerance();
  for (const auto& lane_s_range : lane_s_ranges) {
    const LanePositionResult result =
        road_geometry->ById().GetLane(lane_s_range.lane_id())->ToLanePosition(inertial_position);
    if (result.distance <= linear_tolerance) {
      const double s_position = result.lane_position.s();
      return lane_s_range.s_range().Intersects(SRange(s_position, s_position), linear_tolerance);
    }
  }
  return false;
}

}  // namespace api
}  // namespace maliput
