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

#include <string>
#include <vector>

#include "maliput/api/road_geometry.h"
#include "maliput/api/road_network.h"
#include "maliput/api/segment.h"
#include "maliput/utility/mesh.h"

namespace maliput {
namespace utility {

/// Multitude of parameters for generating an OBJ model of a road surface,
/// with sensible defaults.
struct ObjFeatures {
  /// Maximum distance between rendered vertices, in either s- or r-dimension,
  /// along a lane's surface
  double max_grid_unit{1.0};
  /// Minimum number of vertices, in either s- or r-dimension, along a lane's
  /// surface.
  double min_grid_resolution{5.0};
  /// Draw stripes along lane_bounds() of each lane?
  bool draw_stripes{true};
  /// Draw arrows at start/finish of each lane?
  bool draw_arrows{true};
  /// Draw highlighting swath with lane_bounds() of each lane?
  bool draw_lane_haze{true};
  /// Draw branching at BranchPoints?
  bool draw_branch_points{true};
  /// Draw highlighting of elevation_bounds of each lane?
  bool draw_elevation_bounds{true};
  /// Reduce the amount of vertices from the road by creating
  /// quads big enough which don't violate some tolerance. This could affect
  /// the accuracy of curved roads.
  bool off_grid_mesh_generation{false};
  /// Tolerance for mesh simplification, or the distance from a vertex to an
  /// edge line or to a face plane at which said vertex is considered redundant
  /// (i.e. it is not necessary to further define those geometrical entities),
  /// in meters. If equal to 0, no mesh simplification will take place. If equal
  /// to the road linear tolerance, mesh simplification will be constrained
  /// enough so as to keep mesh geometrical accuracy. If greater than the road
  /// linear tolerance, mesh size reductions will come at the expense of
  /// geometrical accuracy.
  double simplify_mesh_threshold{0.};
  /// Absolute width of stripes
  double stripe_width{0.25};
  /// Absolute elevation (h) of stripes above road surface
  double stripe_elevation{0.05};
  /// Absolute elevation (h) of arrows above road surface
  double arrow_elevation{0.05};
  /// Absolute elevation (h) of lane-haze above road surface
  double lane_haze_elevation{0.02};
  /// Absolute elevation (h) of branch-points above road surface
  double branch_point_elevation{0.5};
  /// Height of rendered branch-point arrows
  double branch_point_height{0.5};
  /// Origin of OBJ coordinates relative to `Inertial`-frame
  api::InertialPosition origin{0., 0., 0.};
  /// ID's of specific segments to be highlighted.  (If non-empty, then the
  /// Segments *not* specified on this list will be rendered as grayed-out.)
  std::vector<api::SegmentId> highlighted_segments;
};

enum class MaterialType {
  Asphalt,
  Lane,
  Marker,
  HBounds,
  BranchPointGlow,
  GrayedAsphalt,
  GrayedLane,
  GrayedMarker,
  Sidewalk
};

/// Material information for built meshes.
struct Material {
  std::string name;
  math::Vector3 diffuse;   /// Kd
  math::Vector3 ambient;   /// Ka
  math::Vector3 specular;  /// Ks
  double shininess;        /// Ns
  double transparency;     /// 1.0 - d

  friend bool operator==(const Material& matA, const Material& matB) { return matA.name == matB.name; }

  friend bool operator==(const Material& matA, const std::string& name) { return matA.name == name; }
};

// Mesh information for all Segments, Lanes, BranchPoints
struct RoadGeometryMesh {
  std::map<std::string, std::pair<mesh::GeoMesh, Material>> asphalt_mesh;
  std::map<std::string, std::pair<mesh::GeoMesh, Material>> grayed_asphalt_mesh;
  std::map<std::string, std::pair<mesh::GeoMesh, Material>> hbounds_mesh;

  std::map<std::string, std::pair<mesh::GeoMesh, Material>> lane_lane_mesh;
  std::map<std::string, std::pair<mesh::GeoMesh, Material>> lane_marker_mesh;
  std::map<std::string, std::pair<mesh::GeoMesh, Material>> lane_grayed_lane_mesh;
  std::map<std::string, std::pair<mesh::GeoMesh, Material>> lane_grayed_marker_mesh;

  std::map<std::string, std::pair<mesh::GeoMesh, Material>> branch_point_mesh;

  std::map<std::string, std::pair<mesh::GeoMesh, Material>> sidewalk_mesh;
};

/// Builds a map of meshes based on `features` properties and the RoadGeometry.
///
/// @param rg the api::RoadGeometry to model.
/// @param features Parameters for constructing the mesh.
/// @param lane_id The ID of the api::Lane to model.
/// @param mesh_material The material to use for the api::Lane.
/// @return A pair with of a mesh and the corresponding material
/// @throws if @p mesh_material is not one of Lane's valid mesh materials
std::pair<mesh::GeoMesh, Material> BuildMesh(const api::RoadGeometry* rg, const ObjFeatures& features,
                                             const api::LaneId& lane_id, const MaterialType& mesh_material);

/// Builds a map of meshes based on `features` properties and the RoadGeometry.
///
/// @param rg The api::RoadGeometry to model.
/// @param features Parameters for constructing the mesh.
/// @param branch_point_id The ID of the api::BranchPoint to model.
/// @param mesh_material The material to use for the api::BranchPoint.
/// @return A pair with of a mesh and the corresponding material
std::pair<mesh::GeoMesh, Material> BuildMesh(const api::RoadGeometry* rg, const ObjFeatures& features,
                                             const api::BranchPointId& branch_point_id,
                                             const MaterialType& mesh_material);

/// Builds a map of meshes based on `features` properties and the RoadGeometry.
///
/// @param rg The api::RoadGeometry to model.
/// @param features Parameters for constructing the mesh.
/// @param segment_id The ID of the api::Segment to model.
/// @param mesh_material The material to use for the api::Segment.
/// @return A pair with of a mesh and the corresponding material
/// @throws if @p mesh_material is not one of Segment's valid mesh materials
std::pair<mesh::GeoMesh, Material> BuildMesh(const api::RoadGeometry* rg, const ObjFeatures& features,
                                             const api::SegmentId& segment_id, const MaterialType& mesh_material);

/// Builds a complete RoadGeometryMesh based on `features` properties and the RoadGeometry.
///
/// @param rg the api::RoadGeometry to model.
/// @param features parameters for constructing the mesh.
/// @return A RoadGeometryMesh with all segment, lane, and branch point meshes
/// @throws if @p mesh_material is not one of BranchPoint's valid mesh materials
RoadGeometryMesh BuildRoadGeometryMesh(const api::RoadGeometry* rg, const ObjFeatures& features);

/// Builds a map of meshes based on `features` properties and the api::RoadGeometry.
///
/// @param road_geometry The api::RoadGeometry to model.
/// @param features Parameters for constructing the mesh.
/// @return A map with the meshes. Keys will be std::string objects in the
/// following list:
///   - asphalt
///   - lane
///   - marker
///   - h_bounds
///   - branch_point
///   - grayed_asphalt
///   - grayed_lane
///   - grayed_marker
///   - sidewalk
std::map<std::string, std::pair<mesh::GeoMesh, Material>> BuildMeshes(const api::RoadGeometry* road_geometry,
                                                                      const ObjFeatures& features);

/// Builds a map of meshes based on `features` properties and the api::RoadNetwork.
///
/// Rules in the api::RoadRulebook will be used to change the direction of the
/// of the lanes and the type of lanes when that information is available.
///
/// @param road_network The api::RoadNetwork to model.
/// @param features Parameters for constructing the mesh.
/// @return A map with the meshes. Keys will be std::string objects in the
/// following list:
///   - asphalt
///   - lane
///   - marker
///   - h_bounds
///   - branch_point
///   - grayed_asphalt
///   - grayed_lane
///   - grayed_marker
///   - sidewalk
///
/// @throws maliput::common::assertion_error When @p road_geometry is nullptr.
std::map<std::string, std::pair<mesh::GeoMesh, Material>> BuildMeshes(const api::RoadNetwork* road_network,
                                                                      const ObjFeatures& features);

/// Generates a Wavefront OBJ model of the road surface of an api::RoadGeometry.
///
/// @param road_geometry  the api::RoadGeometry to model
/// @param dirpath  directory component of the output pathnames
/// @param fileroot  root of the filename component of the output pathnames
/// @param features  parameters for constructing the mesh
///
/// GenerateObjFile actually produces two files:  the first, named
/// [@p dirpath]/[@p fileroot].obj, is a Wavefront OBJ containing the
/// mesh which models the api::RoadGeometry.  The second file is a
/// Wavefront MTL file named [@p dirpath]/[@p fileroot].mtl, containing
/// descriptions of materials referenced by the OBJ file.
///
/// The produced mesh covers the area within the segment-bounds of the
/// road surface described by the RoadGeometry.
///
/// @throws maliput::common::assertion_error When @p road_geometry is nullptr.
/// @throws maliput::common::assertion_error When @p road_geometry->linear_tolerance() is zero.
void GenerateObjFile(const api::RoadGeometry* road_geometry, const std::string& dirpath, const std::string& fileroot,
                     const ObjFeatures& features);

/// Generates a Wavefront OBJ model of the road surface of an api::RoadNetwork.
///
/// Rules in the api::RoadRulebook will be used to change the direction of the
/// lanes and the type of lanes when that information is available.
///
/// @param road_network The api::RoadNetwork to model.
/// @param dirpath Directory component of the output pathnames.
/// @param fileroot Root of the filename component of the output pathnames.
/// @param features Parameters for constructing the mesh.
///
/// GenerateObjFile actually produces two files:  the first, named
/// [@p dirpath]/[@p fileroot].obj, is a Wavefront OBJ containing the
/// mesh which models the api::RoadNetwork. The second file is a
/// Wavefront MTL file named [@p dirpath]/[@p fileroot].mtl, containing
/// descriptions of materials referenced by the OBJ file.
///
/// The produced mesh covers the area within the segment-bounds of the
/// road surface described by the `road_network->road_geometry()`.
///
/// @throws maliput::common::assertion_error When @p road_network is nullptr.
void GenerateObjFile(const api::RoadNetwork* road_network, const std::string& dirpath, const std::string& fileroot,
                     const ObjFeatures& features);

/// Gets a Material based on `material_name` key.
///
/// Possible `material_name` values may be any of the following:
///   - asphalt
///   - lane
///   - marker
///   - h_bounds
///   - branch_point
///   - grayed_asphalt
///   - grayed_lane
///   - grayed_marker
///   - sidewalk
///
/// @param material_name The key to get the material.
/// @return A const Material&
/// @throws std::out_of_range if key is not found.
const Material& GetMaterialByName(const std::string& material_name);

}  // namespace utility
}  // namespace maliput
