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
#include "maliput/utility/generate_obj.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <map>
#include <tuple>
#include <vector>

#include <fmt/ostream.h>

#include "maliput/api/branch_point.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/common/logger.h"
#include "maliput/common/maliput_abort.h"
#include "maliput/common/profiler.h"
#include "maliput/math/vector.h"
#include "maliput/utility/mesh.h"
#include "maliput/utility/mesh_simplification.h"

namespace maliput {
namespace utility {

using mesh::GeoMesh;
using mesh::SimplifyMeshFaces;
using mesh::SrhFace;

namespace {

const std::string kBlandAsphalt("bland_asphalt");
const std::string kLaneHaze("lane_haze");
const std::string kMarkerPaint("marker_paint");
const std::string kHBoundsHaze("h_bounds_haze");
const std::string kBranchPointGlow("branch_point_glow");
const std::string kGrayedBlandAsphalt("grayed_bland_asphalt");
const std::string kGrayedLaneHaze("grayed_lane_haze");
const std::string kGrayedMarkerPaint("grayed_marker_paint");
const std::string kSidewalk("sidewalk");

// This vector holds the properties of different materials. Those properties
// were taken from the original .mtl description that
// lives in GenerateObjFile().
const std::vector<Material> kMaterial{
    {kBlandAsphalt, {0.2, 0.2, 0.2}, {0.1, 0.1, 0.1}, {0.3, 0.3, 0.3}, 10.1, 0.0},
    {kLaneHaze, {0.9, 0.9, 0.9}, {0.9, 0.9, 0.9}, {0.9, 0.9, 0.9}, 10.1, 0.8},
    {kMarkerPaint, {0.8, 0.8, 0.0}, {1.0, 1.0, 0.0}, {1.0, 1.0, 0.5}, 10.1, 0.5},
    {kHBoundsHaze, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, 10.1, 0.8},
    {kBranchPointGlow, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, 10.1, 0.9},
    {kGrayedBlandAsphalt, {0.1, 0.1, 0.1}, {0.2, 0.2, 0.2}, {0.3, 0.3, 0.3}, 10.1, 0.9},
    {kGrayedLaneHaze, {0.9, 0.9, 0.9}, {0.9, 0.9, 0.9}, {0.9, 0.9, 0.9}, 10.1, 0.9},
    {kGrayedMarkerPaint, {0.8, 0.8, 0.0}, {1.0, 1.0, 0.0}, {1.0, 1.0, 0.5}, 10.1, 0.9},
    // TODO(#392): Find an appropriate mesh material configuration.
    {kSidewalk, {0.8, 0.8, 0.0}, {1.0, 1.0, 0.0}, {1.0, 1.0, 0.5}, 10.1, 0.9},
};

std::string FormatVector3AsRow(const math::Vector3& vec) {
  return fmt::format("{} {} {}", std::to_string(vec.x()), std::to_string(vec.y()), std::to_string(vec.z()));
}

std::string FormatMaterial(const Material& mat, int precision) {
  return fmt::format(
      "newmtl {}\n"
      "Ka {}\n"
      "Kd {}\n"
      "Ks {}\n"
      "Ns {}\n"
      "illum 2\n"
      "d {:.{}f}\n",
      mat.name, FormatVector3AsRow(mat.ambient), FormatVector3AsRow(mat.diffuse), FormatVector3AsRow(mat.specular),
      mat.shininess, 1.0 - mat.transparency, precision);
}

// Compute the maximum step in s-coordinates that can approximate the distance
// between its ends in `Inertial`-frame coordinates up to the given
// `grid_unit` (taken as an absolute tolerance).
//
// Let `step_s` be a step in s-coordinates, `s0` a valid s-coordinate in
// `lane` and `grid_unit` the minimum step in s-coordinates to take.
// Starting at the minimum of `grid_unit` and the distance in s-coordinates i.e.
// arc-length, to the `lane`'s end along the center line, increase `step_s`
// until the distance between its ends in `Inertial` space coordinates along the
// center, left most and right most lines of the `lane`'s segment
// surface is bigger than `grid_unit` or the end of the `lane` is reached.
double ComputeSampleStep(const maliput::api::Lane* lane, double s0, double grid_unit) {
  MALIPUT_PROFILE_FUNC();
  MALIPUT_DEMAND(lane != nullptr);

  const double length = lane->length();
  const double min_step = std::min(grid_unit, length - s0);

  const maliput::api::RBounds prev_lane_bounds = lane->lane_bounds(s0);
  const maliput::api::InertialPosition prev_pos_center = lane->ToInertialPosition({s0, 0., 0.});
  const maliput::api::InertialPosition prev_pos_left = lane->ToInertialPosition({s0, prev_lane_bounds.max(), 0.});
  const maliput::api::InertialPosition prev_pos_right = lane->ToInertialPosition({s0, prev_lane_bounds.min(), 0.});

  // Start from minimum step.
  double step_best = min_step;

  while (true) {
    // Make step in travel_with_s direction.
    const double step_proposed = std::min(step_best * 2., length - s0);
    const double s_proposed = s0 + step_proposed;
    const maliput::api::RBounds lane_bounds = lane->lane_bounds(s_proposed);

    const maliput::api::InertialPosition proposed_pos_center_lane = lane->ToInertialPosition({s_proposed, 0., 0.});
    const maliput::api::InertialPosition proposed_pos_left_lane =
        lane->ToInertialPosition({s_proposed, lane_bounds.max(), 0.});
    const maliput::api::InertialPosition proposed_pos_right_lane =
        lane->ToInertialPosition({s_proposed, lane_bounds.min(), 0.});

    // Calculate distance between geo positions for every lane.
    const double distance_from_center = (prev_pos_center.xyz() - proposed_pos_center_lane.xyz()).norm();
    const double distance_from_left = (prev_pos_left.xyz() - proposed_pos_left_lane.xyz()).norm();
    const double distance_from_right = (prev_pos_right.xyz() - proposed_pos_right_lane.xyz()).norm();

    // Check if distance between geo position and local path length ( delta s ) is small enough.
    if (std::fabs(distance_from_center - step_proposed) > grid_unit ||
        std::fabs(distance_from_left - step_proposed) > grid_unit ||
        std::fabs(distance_from_right - step_proposed) > grid_unit) {
      // Tolerance is violated, can't increase step.
      break;
    }
    if (std::fabs(s_proposed - length) < grid_unit) {
      // Clamp the s coordinate since nothing
      // ensures that s_proposed is not negative
      // or a little bit bigger than the lane length.
      step_best = std::max(std::min(0.0, step_proposed), length);
      break;
    }
    // Tolerance is satisfied - increase step 2X in the next iteration and try again.
    step_best = step_proposed;
  }

  step_best = std::min(step_best, length - s0);
  return step_best;
}

// Iterate trough the s-coordinate of the road and generate two triangles
// covering the left and right side of the road, considering the grid_unit
// as a tolerance between patches. Bigger tolerance, worst quality but less
// vertices.
void GenerateOptimizedRoadMesh(GeoMesh* mesh, const api::Lane* lane, double grid_unit, bool use_segment_bounds,
                               const std::function<double(double, double)>& elevation) {
  MALIPUT_PROFILE_FUNC();
  const double s_max = lane->length();
  for (double s0 = 0, s1; s0 < s_max; s0 = s1) {
    const double step_increment = ComputeSampleStep(lane, s0, grid_unit);
    s1 = s0 + step_increment;

    const api::RBounds rb0 = use_segment_bounds ? lane->segment_bounds(s0) : lane->lane_bounds(s0);
    const api::RBounds rb1 = use_segment_bounds ? lane->segment_bounds(s1) : lane->lane_bounds(s1);
    //
    // (s1,r10) o <-- o (s1,r11)
    //          | \   ^
    //          v   \ |
    // (s0,r00) o --> o (s0,r01)
    //
    const double r00 = rb0.max();
    const double r01 = rb0.min();
    const double r10 = rb1.max();
    const double r11 = rb1.min();
    SrhFace first_triangle(
        {{s0, r00, elevation(s0, r00)}, {s0, r01, elevation(s0, r01)}, {s1, r10, elevation(s1, r10)}}, {0., 0., 1.});
    SrhFace second_triangle(
        {{s0, r01, elevation(s0, r01)}, {s1, r11, elevation(s1, r11)}, {s1, r10, elevation(s1, r10)}}, {0., 0., 1.});
    mesh->PushFace(first_triangle.ToGeoFace(lane));
    mesh->PushFace(second_triangle.ToGeoFace(lane));
  }
}

void GeneratePreciseRoadMesh(GeoMesh* mesh, const api::Lane* lane, double grid_unit, bool use_segment_bounds,
                             const std::function<double(double, double)>& elevation) {
  MALIPUT_PROFILE_FUNC();
  const double linear_tolerance = lane->segment()->junction()->road_geometry()->linear_tolerance();
  const double s_max = lane->length();
  for (double s0 = 0, s1; s0 < s_max; s0 = s1) {
    s1 = s0 + grid_unit;
    if (s1 > s_max - linear_tolerance) {
      s1 = s_max;
    }

    const api::RBounds rb0 = use_segment_bounds ? lane->segment_bounds(s0) : lane->lane_bounds(s0);
    const api::RBounds rb1 = use_segment_bounds ? lane->segment_bounds(s1) : lane->lane_bounds(s1);

    // Left side of lane (r >= 0).
    {
      double r00 = 0.;
      double r10 = 0.;
      while ((r00 < rb0.max()) && (r10 < rb1.max())) {
        const double r01 = std::min(r00 + grid_unit, rb0.max());
        const double r11 = std::min(r10 + grid_unit, rb1.max());
        //
        // (s1,r11) o <-- o (s1,r10)       ^ +s
        //          |     ^                |
        //          v     |          +r <--o
        // (s0,r01) o --> * (s0,r00)
        //
        SrhFace srh_face({{s0, r00, elevation(s0, r00)},
                          {s1, r10, elevation(s1, r10)},
                          {s1, r11, elevation(s1, r11)},
                          {s0, r01, elevation(s0, r01)}},
                         {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 += grid_unit;
        r10 += grid_unit;
      }
    }
    // Right side of lane (r <= 0).
    {
      double r00 = 0.;
      double r10 = 0.;
      while ((r00 > rb0.min()) && (r10 > rb1.min())) {
        const double r01 = std::max(r00 - grid_unit, rb0.min());
        const double r11 = std::max(r10 - grid_unit, rb1.min());
        //
        // (s1,r10) o <-- o (s1,r11)  ^ +s
        //          |     ^           |
        //          v     |           o--> -r
        // (s0,r00) * --> o (s0,r01)
        //
        SrhFace srh_face({{s0, r00, elevation(s0, r00)},
                          {s0, r01, elevation(s0, r01)},
                          {s1, r11, elevation(s1, r11)},
                          {s1, r10, elevation(s1, r10)}},
                         {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 -= grid_unit;
        r10 -= grid_unit;
      }
    }
  }
}

// Traverses @p lane, generating a cover of the surface with quads
// (4-vertex faces) which are added to @p mesh.  The quads are quadrilaterals in
// the (s,r) space of the lane.
//
// @param mesh  the GeoMesh which will receive the quads.
// @param lane  the api::Lane to cover with quads.
// @param grid_unit  size of each quad (length of edge in s and r dimensions).
// @param use_segment_bounds  if true, use the lane' segment_bounds()
//        to determine the lateral extent of the coverage; otherwise, use
//        lane_bounds().
// @param elevation a function taking `(s, r)` as parameters and returning
//        the corresponding elevation `h`, to yield a quad vertex `(s, r, h)`.
void CoverLaneWithQuads(GeoMesh* mesh, const api::Lane* lane, double grid_unit, bool use_segment_bounds,
                        const std::function<double(double, double)>& elevation, bool optimize_generation) {
  MALIPUT_PROFILE_FUNC();
  if (optimize_generation) {
    GenerateOptimizedRoadMesh(mesh, lane, grid_unit, use_segment_bounds, elevation);
  } else {
    GeneratePreciseRoadMesh(mesh, lane, grid_unit, use_segment_bounds, elevation);
  }
}

// Adds faces to @p mesh which draw stripes along the lane_bounds() of
// @p lane.
//
// @param mesh  the GeoMesh which will receive the faces
// @param lane  the api::Lane to provide the bounds and surface
// @param grid_unit  longitudinal size (s dimension) of each face
// @param h_offset  h value of each vertex (height above road surface)
// @param stripe_width  width (r dimension) of each stripe
void StripeLaneBounds(GeoMesh* mesh, const api::Lane* lane, double grid_unit, double h_offset, double stripe_width) {
  const double half_stripe = 0.5 * stripe_width;
  const double linear_tolerance = lane->segment()->junction()->road_geometry()->linear_tolerance();
  const double s_max = lane->length();
  for (double s0 = 0, s1; s0 < s_max; s0 = s1) {
    s1 = s0 + grid_unit;
    if (s1 > s_max - linear_tolerance) {
      s1 = s_max;
    }

    api::RBounds rb0 = lane->lane_bounds(s0);
    api::RBounds rb1 = lane->lane_bounds(s1);

    // Left side of lane.
    {
      SrhFace srh_face({{s0, rb0.max() - half_stripe, h_offset},
                        {s1, rb1.max() - half_stripe, h_offset},
                        {s1, rb1.max() + half_stripe, h_offset},
                        {s0, rb0.max() + half_stripe, h_offset}},
                       {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
    // Right side of lane.
    {
      SrhFace srh_face({{s0, rb0.min() - half_stripe, h_offset},
                        {s1, rb1.min() - half_stripe, h_offset},
                        {s1, rb1.min() + half_stripe, h_offset},
                        {s0, rb0.min() + half_stripe, h_offset}},
                       {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
  }
}

// Adds faces to @p mesh which draw a simple triangular arrow in the
// `Lane`-frame of @p lane.  The width of the arrow is fixed at 80% of
// the lane_bounds() of @p lane at the base of the arrow.
//
// @param mesh  the GeoMesh which will receive the faces
// @param lane  the api::Lane to provide the bounds and surface
// @param grid_unit  size of each quad (length of edge in s and r dimensions)
// @param s_offset  longitudinal offset of the base of the arrow from the
//                  beginning (s = 0) of @p lane
// @param s_size  length of the arrow from base to tip
// @param h_offset  h value of each vertex (height above road surface)
void DrawLaneArrow(GeoMesh* mesh, const api::Lane* lane, double grid_unit, double s_offset, double s_size,
                   double h_offset) {
  MALIPUT_PROFILE_FUNC();
  MALIPUT_DEMAND(s_offset >= 0.);
  MALIPUT_DEMAND((s_offset + s_size) <= lane->length());
  const double kRelativeWidth = 0.8;

  const api::RBounds rb0 = lane->lane_bounds(s_offset);

  const int max_num_s_units = static_cast<int>(std::ceil(s_size / grid_unit));

  const double rl_size = rb0.max() * kRelativeWidth;
  const double rr_size = -rb0.min() * kRelativeWidth;
  const int max_num_rl_units = static_cast<int>(std::ceil(rl_size / grid_unit));
  const int max_num_rr_units = static_cast<int>(std::ceil(rr_size / grid_unit));

  const int num_units = std::max(max_num_s_units, std::max(max_num_rl_units, max_num_rr_units));

  MALIPUT_DEMAND(num_units >= 1);
  const double s_unit = s_size / num_units;
  const double rl_unit = rl_size / num_units;
  const double rr_unit = rr_size / num_units;

  int num_r_units = num_units;
  for (int si = 0; si < num_units; ++si) {
    double s0 = s_offset + (si * s_unit);
    double s1 = s_offset + ((si + 1.) * s_unit);
    // Left side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      for (int ri = 0; ri < (num_r_units - 1); ++ri) {
        const double r01 = r00 + rl_unit;
        const double r11 = r10 + rl_unit;
        //
        // (s1,r11) o <-- o (s1,r10)       ^ +s
        //          |     ^                |
        //          v     |          +r <--o
        // (s0,r01) o --> * (s0,r00)
        //
        SrhFace srh_face({{s0, r00, h_offset}, {s1, r10, h_offset}, {s1, r11, h_offset}, {s0, r01, h_offset}},
                         {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 += rl_unit;
        r10 += rl_unit;
      }
      //                o (s1,r10)       ^ +s
      //              / ^                |
      //            /   |          +r <--o
      // (s0,r01) o --> * (s0,r00)
      SrhFace srh_face({{s0, r00, h_offset}, {s1, r10, h_offset}, {s0, r00 + rl_unit, h_offset}}, {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
    // Right side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      for (int ri = 0; ri < (num_r_units - 1); ++ri) {
        const double r01 = r00 - rr_unit;
        const double r11 = r10 - rr_unit;
        //
        // (s1,r10) o <-- o (s1,r11)  ^ +s
        //          |     ^           |
        //          v     |           o--> -r
        // (s0,r00) * --> o (s0,r01)
        //
        SrhFace srh_face({{s0, r00, h_offset}, {s0, r01, h_offset}, {s1, r11, h_offset}, {s1, r10, h_offset}},
                         {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 -= rr_unit;
        r10 -= rr_unit;
      }
      //
      // (s1,r10) o                 ^ +s
      //          | \               |
      //          v   \             o--> -r
      // (s0,r00) * --> o (s0,r01)
      //
      SrhFace srh_face({{s0, r00, h_offset}, {s0, r00 - rr_unit, h_offset}, {s1, r10, h_offset}}, {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }

    num_r_units -= 1;
  }
}

// Marks the start and finish ends of @p lane with arrows, rendered into
// @p mesh.
//
// When the arrow length is smaller than the linear tolerance or it starts /
// ends outside lane boundaries, it is omitted.
//
// @param mesh  the GeoMesh which will receive the arrows
// @param lane  the api::Lane to provide the surface
// @param grid_unit  size of each quad (length of edge in s and r dimensions)
// @param h_offset  h value of each vertex (height above road surface)
void MarkLaneEnds(GeoMesh* mesh, const api::Lane* lane, double grid_unit, double h_offset) {
  MALIPUT_PROFILE_FUNC();
  // To avoid crossing boundaries (and tripping assertions) due to
  // numeric precision issues, we will nudge the arrows inward from
  // the ends of the lanes by the RoadGeometry's linear_tolerance().
  const double nudge = lane->segment()->junction()->road_geometry()->linear_tolerance();
  const double max_length = 0.3 * lane->length();
  // Arrows are sized relative to their respective ends.
  const api::RBounds start_rb = lane->lane_bounds(0.);
  const double start_s_size = std::min(max_length, (start_rb.max() - start_rb.min()));

  const api::RBounds finish_rb = lane->lane_bounds(lane->length());
  const double finish_s_size = std::min(max_length, (finish_rb.max() - finish_rb.min()));

  // Avoid drawing the arrows when its length is shorter than nudge or it would
  // start / end outside lane boundaries.
  if (nudge + start_s_size < lane->length() && start_s_size > nudge) {
    DrawLaneArrow(mesh, lane, grid_unit, 0. + nudge, start_s_size, h_offset);
  }
  if (lane->length() - nudge - finish_s_size > 0. && finish_s_size > nudge) {
    DrawLaneArrow(mesh, lane, grid_unit, lane->length() - finish_s_size - nudge, finish_s_size, h_offset);
  }
}

// Calculates an appropriate grid-unit size for @p lane.
//
// Grid size will be the smaller between:
// - Initial lane width divided by @p min_resolution.
// - Final lane width divided by @p min_resolution.
// - @p lane's length divided by @p min_resolution.
//
// However, the returned grid will never be smaller than @p linear_tolerance.
// Note that @p max_size would be the maximum possible grid unit every time it
// it is bigger than @p linear_tolerance.
double PickGridUnit(const api::Lane* lane, double max_size, double min_resolution, double linear_tolerance) {
  MALIPUT_PROFILE_FUNC();
  double result = max_size;

  const api::RBounds rb0 = lane->lane_bounds(0.);
  const double width0 = std::max(rb0.max() - rb0.min(), linear_tolerance);

  const api::RBounds rb1 = lane->lane_bounds(lane->length());
  const double width1 = std::max(rb1.max() - rb1.min(), linear_tolerance);

  const double length = std::max(lane->length(), linear_tolerance);

  result = std::min(result, width0 / min_resolution);
  result = std::min(result, width1 / min_resolution);
  result = std::min(result, length / min_resolution);
  // Grid unit should not be less than linear_tolerance.
  result = std::max(result, linear_tolerance);

  return result;
}

// Generates a mesh to be used for a BranchPoint
//
// @param elevation   the elevation over the BranchPoint which the mesh /
//                    will be generated
// @param height      the height of the mesh to be generated
// @param lane_end    one of the lane ends within the BranchPoint to be used /
//                    as a reference for its geometry
// @param as_diamond  whether a diamond shape is desired or not
// @param mesh        the mesh to store the created shapes in
void DrawBranch(double elevation, double height, const api::LaneEnd& lane_end, bool as_diamond, GeoMesh* mesh) {
  MALIPUT_PROFILE_FUNC();
  MALIPUT_THROW_UNLESS(mesh != nullptr && elevation >= 0 && height >= 0);

  static const double kWidthFactor = 0.1;
  static const double kTipFactor = 0.1;
  static const double kLengthFactor = 1.0;
  static const double kMaxLengthFraction = 0.4;

  const bool is_end_of_lane = lane_end.end == api::LaneEnd::kStart;
  const double end_s = is_end_of_lane ? 0. : lane_end.lane->length();
  const api::RBounds r_bounds = lane_end.lane->lane_bounds(end_s);

  // TODO(anyone): Remove the assumption that lane centerline is centered with respect to lane_bound().
  const double half_width = (r_bounds.max() - r_bounds.min()) * kWidthFactor * 0.5;
  const double length =
      std::min(kMaxLengthFraction * lane_end.lane->length(), kLengthFactor * (r_bounds.max() - r_bounds.min())) *
      (is_end_of_lane ? 1. : -1);

  const double left_r = half_width * (is_end_of_lane ? 1. : -1);
  const double right_r = -left_r;

  if (as_diamond) {
    SrhFace srh_face({{end_s, 0., elevation - (0.5 * height)},
                      {end_s, right_r, elevation},
                      {end_s, 0., elevation + (0.5 * height)},
                      {end_s, left_r, elevation}},
                     api::LanePosition{(end_s == 0. ? 1. : -1), 0., 0.});
    mesh->PushFace(srh_face.ToGeoFace(lane_end.lane));
  } else {
    SrhFace srh_face1({{end_s, left_r, elevation},
                       {end_s, right_r, elevation},
                       {end_s + length, right_r * kTipFactor, elevation},
                       {end_s + length, left_r * kTipFactor, elevation}},
                      api::LanePosition{0., 0., 1.});
    SrhFace srh_face2({{end_s, 0., elevation - (0.5 * height)},
                       {end_s, 0., elevation + (0.5 * height)},
                       {end_s + length, 0., elevation + (0.5 * kTipFactor * height)},
                       {end_s + length, 0., elevation - (0.5 * kTipFactor * height)}},
                      api::LanePosition{0., (length > 0. ? 1. : -1.), 0.});
    mesh->PushFace(srh_face1.ToGeoFace(lane_end.lane));
    mesh->PushFace(srh_face2.ToGeoFace(lane_end.lane));
  }
}

// Generates a meshes for all BranchPoints in a LaneEndSet
//
// @param elevation   the elevation over the BranchPoint which the mesh /
//                    will be generated
// @param height      the height of the mesh to be generated
// @param set         the set containing the BranchPoints to draw indicators /
//                    for
// @param mesh        the mesh to store the created shapes in
void DrawArrows(double elevation, double height, const api::LaneEndSet* set, GeoMesh* mesh) {
  MALIPUT_PROFILE_FUNC();
  MALIPUT_THROW_UNLESS(set != nullptr && mesh != nullptr);

  for (int i = 0; i < set->size(); ++i) {
    DrawBranch(elevation, height, set->get(i), false /* as_diamond */, mesh);
  }
}

// Renders a BranchPoint @p branch_point as a collection of pointy
// arrows for each branch.  @p base_elevation is the desired elevation
// of the center of the rendering (above the road surface), and
// @p height is the vertical size of rendering.  The actual elevation
// may be raised in order to avoid overlapping other nearby
// BranchPoints.  @p mesh is the mesh into which the rendering occurs.
// @p previous_centers is a list of the `Inertial`-frame positions of the
// centers of previously rendered BranchPoints (in order to avoid
// overlaps with them); this list will be updated with the rendered
// center of this BranchPoint.
void RenderBranchPoint(const api::BranchPoint* const branch_point, const double base_elevation, const double height,
                       GeoMesh* mesh, std::vector<api::InertialPosition>* previous_centers) {
  MALIPUT_PROFILE_FUNC();
  if ((branch_point->GetASide()->size() == 0) && (branch_point->GetBSide()->size() == 0)) {
    // No branches?  Odd, but, oh, well... nothing to do here.
    return;
  }

  // Arbitrarily pick one of the LaneEnds in the BranchPoint as a reference
  // for its geometry (e.g., *where* is the BranchPoint).
  const api::LaneEnd reference_end =
      (branch_point->GetASide()->size() > 0) ? branch_point->GetASide()->get(0) : branch_point->GetBSide()->get(0);
  const double reference_end_s = (reference_end.end == api::LaneEnd::kStart) ? 0. : reference_end.lane->length();
  const api::RBounds reference_bounds = reference_end.lane->lane_bounds(reference_end_s);
  const double sr_margin = reference_bounds.max() - reference_bounds.min();
  const double h_margin = height;

  // Choose an elevation that keeps this BranchPoint out of the way
  // of previously rendered BranchPoints.
  double elevation = base_elevation;
  bool has_conflict = true;
  while (has_conflict) {
    // Calculate center in `Inertial`-frame with current elevation.
    const api::LanePosition center_srh((reference_end.end == api::LaneEnd::kStart) ? 0. : reference_end.lane->length(),
                                       0., elevation);
    const api::Rotation orientation = reference_end.lane->GetOrientation(center_srh);
    const api::InertialPosition center_xyz = reference_end.lane->ToInertialPosition(center_srh);

    has_conflict = false;
    // Compare center against every already-rendered center....
    // If distance in sr-plane is too close and distance along h-axis is
    // too close, then increase elevation and try again.
    for (const api::InertialPosition& previous_xyz : *previous_centers) {
      const math::Vector3 delta_srh = orientation.matrix().transpose() * (previous_xyz.xyz() - center_xyz.xyz());
      if ((math::Vector2(delta_srh.x(), delta_srh.y()).norm() < sr_margin) && (std::abs(delta_srh.z()) < h_margin)) {
        has_conflict = true;
        elevation += height;
        break;
      }
    }

    if (!has_conflict) {
      previous_centers->emplace_back(center_xyz);
    }
  }

  // Finally, draw the BranchPoint as:
  // - a single vertical diamond, facing into the lane of reference_end;
  // - for each branch (LaneEnd), an arrow formed from a pair of very
  //   pointy trapezoids (one in the sr-plane, one in the sh-plane) pointing
  //   into the lane.

  DrawBranch(elevation, height, reference_end, true /* as_diamond */, mesh);
  DrawArrows(elevation, height, branch_point->GetASide(), mesh);
  DrawArrows(elevation, height, branch_point->GetBSide(), mesh);
}

GeoMesh SimplifyMesh(const GeoMesh& mesh, const ObjFeatures& features) {
  MALIPUT_PROFILE_FUNC();
  if (features.simplify_mesh_threshold == 0.) {
    return mesh;  // Passes given mesh unmodified.
  }
  return SimplifyMeshFaces(mesh, features.simplify_mesh_threshold);
}

// Render the `segment`'s lanes if any, to add the outcome to `asphalt_mesh`, `lane_mesh`, `marker_mesh` and
// `h_bounds_mesh`.
// @param segment Contains the lanes to be rendered.
// @param features Holds parameters for generating an OBJ model.
// @param asphalt_mesh GeoFaces related to the asphalt mesh.
// @param lane_mesh GeoFaces related to the lane mesh.
// @param marker_mesh GeoFaces related to the maker mesh.
// @param h_bounds_mesh GeoFaces related to the elevation boundary mesh.
// @param sidewalk_mesh GeoFaces related to lanes that belong to a sidewalk mesh.
//
// @throw maliput::common::assertion_error When `segment` is nullptr.
// @throw maliput::common::assertion_error When '`segment`->junction()' is nullptr.
// @throw maliput::common::assertion_error When '`segment`->junction()->road_geometry()' is nullptr.

// TODO(#392): Receive the RoadRulebook pointer to modify the mesh creation accordingly.
void RenderSegment(const api::Segment* segment, const ObjFeatures& features, GeoMesh* asphalt_mesh, GeoMesh* lane_mesh,
                   GeoMesh* marker_mesh, GeoMesh* h_bounds_mesh, GeoMesh* sidewalk_mesh) {
  MALIPUT_PROFILE_FUNC();
  MALIPUT_THROW_UNLESS(segment != nullptr);
  MALIPUT_THROW_UNLESS(segment->junction() != nullptr);
  MALIPUT_THROW_UNLESS(segment->junction()->road_geometry() != nullptr);
  if (segment->num_lanes() == 0) {
    maliput::log()->trace("The are no lanes to be rendered in Segment ID: {}.", segment->id().string());
    return;
  }
  const double linear_tolerance = segment->junction()->road_geometry()->linear_tolerance();
  const double base_grid_unit =
      features.off_grid_mesh_generation
          ? linear_tolerance
          : PickGridUnit(segment->lane(0), features.max_grid_unit, features.min_grid_resolution, linear_tolerance);
  {
    // Lane 0 should be as good as any other for segment-bounds.
    GeoMesh segment_mesh;
    // clang-format off
    CoverLaneWithQuads(&segment_mesh, segment->lane(0), base_grid_unit, true /*use_segment_bounds*/,
                       [](double, double) { return 0.; }, features.off_grid_mesh_generation);
    // clang-format on
    asphalt_mesh->AddFacesFrom(SimplifyMesh(segment_mesh, features));
  }

  if (features.draw_elevation_bounds) {
    maliput::log()->trace("Drawing elevation bounds");
    GeoMesh upper_h_bounds_mesh, lower_h_bounds_mesh;
    // clang-format off
    CoverLaneWithQuads(&upper_h_bounds_mesh, segment->lane(0), base_grid_unit, true /*use_segment_bounds*/,
                       [&segment](double s, double r) { return segment->lane(0)->elevation_bounds(s, r).max(); },
                       features.off_grid_mesh_generation);
    CoverLaneWithQuads(&lower_h_bounds_mesh, segment->lane(0), base_grid_unit, true /*use_segment_bounds*/,
                       [&segment](double s, double r) { return segment->lane(0)->elevation_bounds(s, r).min(); },
                       features.off_grid_mesh_generation);
    // clang-format on
    h_bounds_mesh->AddFacesFrom(SimplifyMesh(upper_h_bounds_mesh, features));
    h_bounds_mesh->AddFacesFrom(SimplifyMesh(lower_h_bounds_mesh, features));
  }
  for (int li = 0; li < segment->num_lanes(); ++li) {
    const api::Lane* lane = segment->lane(li);
    if (!lane) {
      continue;
    }
    maliput::log()->trace("Creating meshes for lane id {}", lane->id().string());
    const double grid_unit = PickGridUnit(lane, features.max_grid_unit, features.min_grid_resolution, linear_tolerance);
    if (features.draw_lane_haze) {
      GeoMesh haze_mesh;
      // clang-format off
      CoverLaneWithQuads(&haze_mesh, lane, base_grid_unit, false /*use_segment_bounds*/,
                         [&features](double, double) { return features.lane_haze_elevation; },
                         features.off_grid_mesh_generation);
      // clang-format on
      lane_mesh->AddFacesFrom(SimplifyMesh(haze_mesh, features));
    }
    if (features.draw_stripes) {
      GeoMesh stripes_mesh;
      StripeLaneBounds(&stripes_mesh, lane, grid_unit, features.stripe_elevation, features.stripe_width);
      marker_mesh->AddFacesFrom(SimplifyMesh(stripes_mesh, features));
    }
    if (features.draw_arrows) {
      GeoMesh arrows_mesh;
      MarkLaneEnds(&arrows_mesh, lane, grid_unit, features.arrow_elevation);
      marker_mesh->AddFacesFrom(SimplifyMesh(arrows_mesh, features));
    }
  }
}

bool IsSegmentRenderedNormally(const api::SegmentId& id, const std::vector<api::SegmentId>& highlights) {
  if (highlights.empty()) {
    return true;
  }
  for (const api::SegmentId& highlighted_id : highlights) {
    if (id == highlighted_id) {
      return true;
    }
  }
  return false;
}

// Generates an id for the lane mesh
//
// @param id   the id of the lane
std::string LaneKey(const api::LaneId& id) { return "lane_" + id.string(); }

// Generates an id for the marker mesh
//
// @param id   the id of the lane
std::string MarkerKey(const api::LaneId& id) { return "marker_" + id.string(); }

// Generates an id for the grayed lane mesh
//
// @param id   the id of the lane
std::string GrayedLaneKey(const api::LaneId& id) { return "grayed_lane_" + id.string(); }

// Generates an id for the grayed marker mesh
//
// @param id   the id of the lane
std::string GrayedMarkerKey(const api::LaneId& id) { return "grayed_marker_" + id.string(); }

// Generates an id for the branch point mesh
//
// @param id   the id of the branch point
std::string BranchPointKey(const api::BranchPointId& id) { return "branch_point_" + id.string(); }

}  // namespace

Material GetMaterialFromMesh(const MaterialType mesh_material) {
  switch (mesh_material) {
    case MaterialType::Asphalt:
      return GetMaterialByName(kBlandAsphalt);
    case MaterialType::Lane:
      return GetMaterialByName(kLaneHaze);
    case MaterialType::Marker:
      return GetMaterialByName(kMarkerPaint);
    case MaterialType::HBounds:
      return GetMaterialByName(kHBoundsHaze);
    case MaterialType::BranchPointGlow:
      return GetMaterialByName(kBranchPointGlow);
    case MaterialType::GrayedAsphalt:
      return GetMaterialByName(kGrayedBlandAsphalt);
    case MaterialType::GrayedLane:
      return GetMaterialByName(kGrayedLaneHaze);
    case MaterialType::GrayedMarker:
      return GetMaterialByName(kGrayedMarkerPaint);
    case MaterialType::Sidewalk:
      return GetMaterialByName(kSidewalk);
  }
  MALIPUT_THROW_MESSAGE("mesh_material is unrecognized.");
}

std::pair<mesh::GeoMesh, Material> BuildMesh(const api::RoadGeometry* rg, const ObjFeatures& features,
                                             const api::LaneId& lane_id, const MaterialType& mesh_material) {
  MALIPUT_PROFILE("BuildMesh: For lanes");
  MALIPUT_DEMAND(rg != nullptr);
  MALIPUT_THROW_UNLESS(mesh_material != MaterialType::BranchPointGlow);

  GeoMesh mesh;
  const api::RoadGeometry::IdIndex& road_index = rg->ById();
  const api::Lane* lane = road_index.GetLane(lane_id);
  const api::Segment* segment = lane->segment();
  const double linear_tolerance = segment->junction()->road_geometry()->linear_tolerance();
  const double base_grid_unit =
      features.off_grid_mesh_generation
          ? linear_tolerance
          : PickGridUnit(lane, features.max_grid_unit, features.min_grid_resolution, linear_tolerance);

  switch (mesh_material) {
    case MaterialType::Asphalt:
    case MaterialType::GrayedAsphalt: {
      GeoMesh segment_mesh;
      // clang-format off
      CoverLaneWithQuads(&segment_mesh, lane, base_grid_unit, false /* use_lane_bounds */,
                         [](double, double) { return 0.; }, features.off_grid_mesh_generation);
      // clang-format on
      mesh.AddFacesFrom(SimplifyMesh(segment_mesh, features));
      break;
    }
    case MaterialType::Lane:
    case MaterialType::GrayedLane: {
      if (features.draw_lane_haze) {
        GeoMesh haze_mesh;
        // clang-format off
        CoverLaneWithQuads(&haze_mesh, lane, base_grid_unit, false /* use_lane_bounds */,
                           [&features](double, double) { return features.lane_haze_elevation; },
                           features.off_grid_mesh_generation);
        // clang-format on
        mesh.AddFacesFrom(SimplifyMesh(haze_mesh, features));
      }
      break;
    }
    case MaterialType::Marker:
    case MaterialType::GrayedMarker: {
      const double grid_unit =
          PickGridUnit(lane, features.max_grid_unit, features.min_grid_resolution, linear_tolerance);
      if (features.draw_stripes) {
        GeoMesh stripes_mesh;
        StripeLaneBounds(&stripes_mesh, lane, grid_unit, features.stripe_elevation, features.stripe_width);
        mesh.AddFacesFrom(SimplifyMesh(stripes_mesh, features));
      }
      if (features.draw_arrows) {
        GeoMesh arrows_mesh;
        MarkLaneEnds(&arrows_mesh, lane, grid_unit, features.arrow_elevation);
        mesh.AddFacesFrom(SimplifyMesh(arrows_mesh, features));
      }
      break;
    }
    case MaterialType::HBounds: {
      if (features.draw_elevation_bounds) {
        GeoMesh upper_h_bounds_mesh, lower_h_bounds_mesh;
        // clang-format off
        CoverLaneWithQuads(&upper_h_bounds_mesh, lane, base_grid_unit, false /* use_lane_bounds */,
                           [&lane](double s, double r) { return lane->elevation_bounds(s, r).max(); },
                           features.off_grid_mesh_generation);
        CoverLaneWithQuads(&lower_h_bounds_mesh, lane, base_grid_unit, false /* use_lane_bounds */,
                           [&lane](double s, double r) { return lane->elevation_bounds(s, r).min(); },
                           features.off_grid_mesh_generation);
        // clang-format on
        mesh.AddFacesFrom(SimplifyMesh(upper_h_bounds_mesh, features));
        mesh.AddFacesFrom(SimplifyMesh(lower_h_bounds_mesh, features));
      }
      break;
    }
    case MaterialType::BranchPointGlow: {
      // Shouldn't occur due to above assertion, but handle anyway to quash warnings
      MALIPUT_THROW_MESSAGE("BranchPointGlow is not a valid Mesh Material type within this function call.");
      break;
    }
    case MaterialType::Sidewalk: {
      // TODO(#392): Implement this code path.
      MALIPUT_THROW_MESSAGE("Sidewalk is not implemented yet.");
      break;
    }
  }

  return std::make_pair(std::move(mesh), GetMaterialFromMesh(mesh_material));
}

std::pair<mesh::GeoMesh, Material> BuildMesh(const api::RoadGeometry* rg, const ObjFeatures& features,
                                             const api::BranchPointId& branch_point_id,
                                             const MaterialType& mesh_material) {
  MALIPUT_PROFILE("BuildMesh: For BranchPoints");
  MALIPUT_DEMAND(rg != nullptr);
  MALIPUT_THROW_UNLESS(mesh_material == MaterialType::BranchPointGlow);

  GeoMesh mesh;
  const api::RoadGeometry::IdIndex& road_index = rg->ById();
  const api::BranchPoint* branch_point = road_index.GetBranchPoint(branch_point_id);
  const api::LaneEnd reference_end =
      (branch_point->GetASide()->size() > 0) ? branch_point->GetASide()->get(0) : branch_point->GetBSide()->get(0);
  double elevation = features.branch_point_elevation;
  double height = features.branch_point_height;

  DrawBranch(elevation, height, reference_end, true /* as_diamond */, &mesh);
  DrawArrows(elevation, height, branch_point->GetASide(), &mesh);
  DrawArrows(elevation, height, branch_point->GetBSide(), &mesh);

  return std::make_pair(std::move(mesh), GetMaterialFromMesh(mesh_material));
}

std::pair<mesh::GeoMesh, Material> BuildMesh(const api::RoadGeometry* rg, const ObjFeatures& features,
                                             const api::SegmentId& segment_id, const MaterialType& mesh_material) {
  MALIPUT_PROFILE("BuildMesh: For Segments");
  MALIPUT_DEMAND(rg != nullptr);
  MALIPUT_THROW_UNLESS(mesh_material == MaterialType::Asphalt || mesh_material == MaterialType::GrayedAsphalt);

  GeoMesh mesh;
  const api::RoadGeometry::IdIndex& road_index = rg->ById();
  const api::Segment* segment = road_index.GetSegment(segment_id);
  const double linear_tolerance = segment->junction()->road_geometry()->linear_tolerance();
  const double base_grid_unit =
      features.off_grid_mesh_generation
          ? linear_tolerance
          : PickGridUnit(segment->lane(0), features.max_grid_unit, features.min_grid_resolution, linear_tolerance);

  GeoMesh segment_mesh;
  // clang-format off
  CoverLaneWithQuads(&segment_mesh, segment->lane(0), base_grid_unit, true /* use_segment_bounds */,
                     [](double, double) { return 0.; }, features.off_grid_mesh_generation);
  // clang-format on
  mesh.AddFacesFrom(SimplifyMesh(segment_mesh, features));

  Material material = GetMaterialFromMesh(mesh_material);

  return std::make_pair(std::move(mesh), material);
}

RoadGeometryMesh BuildRoadGeometryMesh(const api::RoadGeometry* rg, const ObjFeatures& features) {
  MALIPUT_PROFILE_FUNC();
  RoadGeometryMesh meshes;

  GeoMesh asphalt_mesh;
  GeoMesh lane_mesh;
  GeoMesh marker_mesh;
  GeoMesh h_bounds_mesh;
  GeoMesh branch_point_mesh;

  GeoMesh grayed_asphalt_mesh;
  GeoMesh grayed_lane_mesh;
  GeoMesh grayed_marker_mesh;

  GeoMesh sidewalk_mesh;

  maliput::log()->trace("Generating RoadGeometry's meshes...");

  for (int ji = 0; ji < rg->num_junctions(); ++ji) {
    const api::Junction* junction = rg->junction(ji);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      if (IsSegmentRenderedNormally(segment->id(), features.highlighted_segments)) {
        RenderSegment(segment, features, &asphalt_mesh, &lane_mesh, &marker_mesh, &h_bounds_mesh, &sidewalk_mesh);
        for (int li = 0; li < segment->num_lanes(); ++li) {
          const api::Lane* lane = segment->lane(li);
          meshes.lane_lane_mesh[LaneKey(lane->id())] = BuildMesh(rg, features, lane->id(), MaterialType::Lane);
          meshes.lane_marker_mesh[MarkerKey(lane->id())] = BuildMesh(rg, features, lane->id(), MaterialType::Marker);
        }
      } else {
        RenderSegment(segment, features, &grayed_asphalt_mesh, &grayed_lane_mesh, &grayed_marker_mesh, &h_bounds_mesh,
                      &sidewalk_mesh);
        for (int li = 0; li < segment->num_lanes(); ++li) {
          const api::Lane* lane = segment->lane(li);
          meshes.lane_grayed_lane_mesh[GrayedLaneKey(lane->id())] =
              BuildMesh(rg, features, lane->id(), MaterialType::GrayedLane);
          meshes.lane_grayed_marker_mesh[GrayedMarkerKey(lane->id())] =
              BuildMesh(rg, features, lane->id(), MaterialType::GrayedMarker);
        }
      }
    }
  }

  if (features.draw_branch_points) {
    for (int bpi = 0; bpi < rg->num_branch_points(); ++bpi) {
      const api::BranchPoint* branch_point = rg->branch_point(bpi);
      meshes.branch_point_mesh[BranchPointKey(branch_point->id())] =
          BuildMesh(rg, features, branch_point->id(), MaterialType::BranchPointGlow);
    }
  }

  meshes.asphalt_mesh["asphalt"] = std::make_pair(std::move(asphalt_mesh), GetMaterialByName(kBlandAsphalt));
  meshes.grayed_asphalt_mesh["grayed_asphalt"] =
      std::make_pair(std::move(grayed_asphalt_mesh), GetMaterialByName(kGrayedBlandAsphalt));
  meshes.hbounds_mesh["h_bounds"] = std::make_pair(std::move(h_bounds_mesh), GetMaterialByName(kHBoundsHaze));
  meshes.sidewalk_mesh["sidewalk"] = std::make_pair(std::move(sidewalk_mesh), GetMaterialByName(kSidewalk));

  maliput::log()->trace("Meshes generation completed.");
  return meshes;
}

std::map<std::string, std::pair<mesh::GeoMesh, Material>> BuildMeshes(const api::RoadGeometry* rg,
                                                                      const ObjFeatures& features) {
  MALIPUT_PROFILE_FUNC();
  MALIPUT_THROW_UNLESS(rg != nullptr);
  maliput::log()->trace("Building Meshes for RoadGeometry id {}...", rg->id().string());

  GeoMesh asphalt_mesh;
  GeoMesh lane_mesh;
  GeoMesh marker_mesh;
  GeoMesh h_bounds_mesh;
  GeoMesh branch_point_mesh;

  GeoMesh grayed_asphalt_mesh;
  GeoMesh grayed_lane_mesh;
  GeoMesh grayed_marker_mesh;

  // TODO(#392): Modify RenderSegment to get the RoadRulebook pointer and
  // modify the mesh generation accordingly.
  GeoMesh sidewalk_mesh;

  // TODO(agalbachicar)   Check features with respect to rg tolerance
  //                      properties.

  // Walk the network.
  for (int ji = 0; ji < rg->num_junctions(); ++ji) {
    const api::Junction* junction = rg->junction(ji);
    if (!junction) {
      continue;
      maliput::log()->trace("Visiting junction id {}", junction->id().string());
    }
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      if (!segment) {
        continue;
      }
      maliput::log()->trace("Rendering segment id {}", segment->id().string());
      // TODO(maddog@tri.global)  Id's need well-defined comparison semantics.
      if (IsSegmentRenderedNormally(segment->id(), features.highlighted_segments)) {
        RenderSegment(segment, features, &asphalt_mesh, &lane_mesh, &marker_mesh, &h_bounds_mesh, &sidewalk_mesh);
      } else {
        RenderSegment(segment, features, &grayed_asphalt_mesh, &grayed_lane_mesh, &grayed_marker_mesh, &h_bounds_mesh,
                      &sidewalk_mesh);
      }
    }
  }

  if (features.draw_branch_points) {
    std::vector<api::InertialPosition> rendered_centers;
    for (int bpi = 0; bpi < rg->num_branch_points(); ++bpi) {
      const api::BranchPoint* branch_point = rg->branch_point(bpi);
      if (!branch_point) {
        continue;
      }
      RenderBranchPoint(branch_point, features.branch_point_elevation, features.branch_point_height, &branch_point_mesh,
                        &rendered_centers);
    }
  }

  std::map<std::string, std::pair<mesh::GeoMesh, Material>> meshes;
  meshes["asphalt"] = std::make_pair(std::move(asphalt_mesh), GetMaterialByName(kBlandAsphalt));
  meshes["lane"] = std::make_pair(std::move(lane_mesh), GetMaterialByName(kLaneHaze));
  meshes["marker"] = std::make_pair(std::move(marker_mesh), GetMaterialByName(kMarkerPaint));
  meshes["h_bounds"] = std::make_pair(std::move(h_bounds_mesh), GetMaterialByName(kHBoundsHaze));
  meshes["branch_point"] = std::make_pair(std::move(branch_point_mesh), GetMaterialByName(kBranchPointGlow));
  meshes["grayed_asphalt"] = std::make_pair(std::move(grayed_asphalt_mesh), GetMaterialByName(kGrayedBlandAsphalt));
  meshes["grayed_lane"] = std::make_pair(std::move(grayed_lane_mesh), GetMaterialByName(kGrayedLaneHaze));
  meshes["grayed_marker"] = std::make_pair(std::move(grayed_marker_mesh), GetMaterialByName(kGrayedMarkerPaint));
  meshes["sidewalk"] = std::make_pair(std::move(sidewalk_mesh), GetMaterialByName(kSidewalk));
  return meshes;
}

std::map<std::string, std::pair<mesh::GeoMesh, Material>> BuildMeshes(const api::RoadNetwork* road_network,
                                                                      const ObjFeatures& features) {
  MALIPUT_THROW_UNLESS(road_network != nullptr);
  return BuildMeshes(road_network->road_geometry(), features);
}

void GenerateObjFile(const api::RoadGeometry* rg, const std::string& dirpath, const std::string& fileroot,
                     const ObjFeatures& features) {
  MALIPUT_PROFILE_FUNC();
  MALIPUT_THROW_UNLESS(rg != nullptr);

  std::map<std::string, std::pair<mesh::GeoMesh, Material>> meshes = BuildMeshes(rg, features);
  const GeoMesh& asphalt_mesh = meshes["asphalt"].first;
  const GeoMesh& lane_mesh = meshes["lane"].first;
  const GeoMesh& marker_mesh = meshes["marker"].first;
  const GeoMesh& h_bounds_mesh = meshes["h_bounds"].first;
  const GeoMesh& branch_point_mesh = meshes["branch_point"].first;
  const GeoMesh& grayed_asphalt_mesh = meshes["grayed_asphalt"].first;
  const GeoMesh& grayed_lane_mesh = meshes["grayed_lane"].first;
  const GeoMesh& grayed_marker_mesh = meshes["grayed_marker"].first;
  const GeoMesh& sidewalk_mesh = meshes["sidewalk"].first;

  const std::string obj_filename = fileroot + ".obj";
  const std::string mtl_filename = fileroot + ".mtl";

  const double linear_tolerance = rg->linear_tolerance();
  const int precision = std::max(0., std::ceil(std::log10(std::sqrt(3.) * 5.) - std::log10(linear_tolerance)));

  // Create the requested OBJ file.
  {
    // Figure out the fixed-point precision necessary to render OBJ vertices
    // with enough precision relative to linear_tolerance().
    //
    // Given linear_tolerance ε, we conservatively want to bound the rendering
    // error per component to `ε / (sqrt(3) * 10)`.  The `sqrt(3)` is
    // because the worst-case error in total 3-space distance is `sqrt(3)`
    // times the per-component error.  The `10` is a fudge-factor to ensure
    // that the "rendering error in an OBJ vertex with respect to the
    // maliput-expressed value" is within 10% of the "error-bound between
    // the maliput-expressed position and the underlying ground-truth".
    // In other words, we're aiming for the rendered vertex to be within
    // 110% ε of the ground-truth position.
    //
    // The bound on error due to rounding to `n` places is `0.5 * 10^(-n)`,
    // so we want `n` such that `0.5 * 10^(-n) < ε / (sqrt(3) * 10)`.
    // This yields:  `n > log10(sqrt(3) * 5) - log10(ε)`.
    MALIPUT_THROW_UNLESS(linear_tolerance > 0.);

    std::ofstream os(dirpath + "/" + obj_filename, std::ios::binary);
    fmt::print(os,
               R"X(# GENERATED BY maliput::utility::GenerateObjFile()
#
# DON'T BE A HERO.  Do not edit by hand.

mtllib {}
)X",
               mtl_filename);
    int vertex_index_offset = 0;
    int normal_index_offset = 0;
    std::tie(vertex_index_offset, normal_index_offset) =
        asphalt_mesh.EmitObj(os, kBlandAsphalt, precision, features.origin, vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        lane_mesh.EmitObj(os, kLaneHaze, precision, features.origin, vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        marker_mesh.EmitObj(os, kMarkerPaint, precision, features.origin, vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) = branch_point_mesh.EmitObj(
        os, kBranchPointGlow, precision, features.origin, vertex_index_offset, normal_index_offset);

    std::tie(vertex_index_offset, normal_index_offset) = grayed_asphalt_mesh.EmitObj(
        os, kGrayedBlandAsphalt, precision, features.origin, vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) = grayed_lane_mesh.EmitObj(
        os, kGrayedLaneHaze, precision, features.origin, vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) = grayed_marker_mesh.EmitObj(
        os, kGrayedMarkerPaint, precision, features.origin, vertex_index_offset, normal_index_offset);

    std::tie(vertex_index_offset, normal_index_offset) =
        h_bounds_mesh.EmitObj(os, kHBoundsHaze, precision, features.origin, vertex_index_offset, normal_index_offset);

    std::tie(vertex_index_offset, normal_index_offset) =
        sidewalk_mesh.EmitObj(os, kSidewalk, precision, features.origin, vertex_index_offset, normal_index_offset);
  }

  // Create the MTL file referenced by the OBJ file.
  {
    std::ofstream os(dirpath + "/" + mtl_filename, std::ios::binary);
    os << "# GENERATED BY maliput::utility::GenerateObjFile()\n"
       << "# DON'T BE A HERO.  Do not edit by hand.\n\n";
    for (const auto& matPair : meshes) {
      // TODO(#392): Implement me.
      if (matPair.first == "sidewalk") {
        continue;
      }
      const Material& mat = matPair.second.second;
      os << FormatMaterial(mat, precision);
    }
  }
}

// TODO(#392): Requires proper implementation.
void GenerateObjFile(const api::RoadNetwork* road_network, const std::string& dirpath, const std::string& fileroot,
                     const ObjFeatures& features) {
  MALIPUT_THROW_UNLESS(road_network != nullptr);
  GenerateObjFile(road_network->road_geometry(), dirpath, fileroot, features);
}

const Material& GetMaterialByName(const std::string& material_name) {
  auto material = std::find(kMaterial.cbegin(), kMaterial.cend(), material_name);
  MALIPUT_THROW_UNLESS(material != kMaterial.cend());
  return *material;
}

}  // namespace utility
}  // namespace maliput
