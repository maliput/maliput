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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <set>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "maliput/common/maliput_hash.h"
#include "maliput/math/vector.h"
#include "maliput/utility/mesh.h"

namespace maliput {
namespace utility {
namespace mesh {

/// Let \f$B\f$ be a plane in the 3D Inertial Frame defined by a point \f$p\f$ and a normal
/// non-zero vector \f$n\f$, and let \f$q\f$ be another point in the 3D Inertial Frame. This method
/// returns the Euclidean distance of \f$q\f$ to \f$B\f$.
/// @param n Is a vector normal to the plane.
/// @param p Is a coordinate in the plane.
/// @param q Is a coordinate out of the plane.
/// @return The Euclidean distance of `q` point to the plane \f$B\f$.
/// @pre The norm of @p n is different from zero.
/// @warning If any of the preconditions is not met, this function will
///          abort execution.
double DistanceToAPlane(const math::Vector3 n, const math::Vector3 p, const math::Vector3 q);

/// Let \f$F(t) = P + Rt\f$ be a parametric line in the 3D Inertial Frame defined by a point \f$P\f$
/// and a vector \f$R\f$, and let \f$Q\f$ be another point in the 3D Inertial Frame. This function
/// returns the Euclidean distance of \f$Q\f$ to \f$F(t)\f$.
/// @param p Is the origin of the parametric line.
/// @param r Is the direction of the parametric line.
/// @param q Is a coordinate out of the line.
/// @return The Euclidean distance of `q` point to the line \f$F(t)\f$.
double DistanceToALine(const math::Vector3& p, const math::Vector3& r, const math::Vector3& q);

/// Index for a directed edge in a GeoMesh.
struct DirectedEdgeIndex {
  /// Returns this edge but reversed.
  DirectedEdgeIndex reverse() const { return {end_vertex_index, start_vertex_index}; }

  int start_vertex_index{-1};  ///< Index of start vertex, or -1
                               ///  to mark index as invalid.
  int end_vertex_index{-1};    ///< Index of end vertex, or -1 to
                               ///  mark index as invalid.
};

inline bool operator==(const DirectedEdgeIndex& lhs, const DirectedEdgeIndex& rhs) {
  return (lhs.start_vertex_index == rhs.start_vertex_index && lhs.end_vertex_index == rhs.end_vertex_index);
}

inline bool operator!=(const DirectedEdgeIndex& lhs, const DirectedEdgeIndex& rhs) { return !(lhs == rhs); }

/// Implements the @ref hash_append concept.
template <class HashAlgorithm>
void hash_append(HashAlgorithm& hasher, const DirectedEdgeIndex& item) noexcept {
  using maliput::common::hash_append;
  hash_append(hasher, item.start_vertex_index);
  hash_append(hasher, item.end_vertex_index);
}

/// Index for a face edge in a GeoMesh.
struct FaceEdgeIndex {
  int face_index{-1};  ///< Index of the face the edge belongs to, or
                       ///  -1 to mark index as invalid.
  int edge_index{-1};  ///< Index of the edge, corresponding to its
                       ///  start vertex index in the face vertices
                       ///  sequence (assumed to be defined in a
                       ///  counter-clockwise fashion), or -1 to mark
                       ///  index as invalid.
};

inline bool operator==(const FaceEdgeIndex& lhs, const FaceEdgeIndex& rhs) {
  return (lhs.face_index == rhs.face_index && lhs.edge_index == rhs.edge_index);
}

inline bool operator!=(const FaceEdgeIndex& lhs, const FaceEdgeIndex& rhs) { return !(lhs == rhs); }

/// The inverse of the mapping from face edges indices to their
/// associated directed edge indices.
/// @see ComputeInverseFaceEdgeMap
using InverseFaceEdgeMap = std::unordered_map<DirectedEdgeIndex, FaceEdgeIndex, maliput::common::DefaultHash>;

/// Computes the inverse of the mapping from face edges indices to their
/// associated directed edge indices for the given @p faces collection.
/// @pre Mapping from directed edges to face edges is 1-to-1. In other
///      words, any given pair of vertices can be shared by two faces at
///      most. This implies that the mesh is well-oriented, such that any
///      adjacent have the common edge in opposite directions.
/// @warning If any of the preconditions is not met, this function will
///          abort execution.
InverseFaceEdgeMap ComputeInverseFaceEdgeMap(const std::vector<IndexFace>& faces);

/// A mapping from each IndexFace index in a given GeoMesh to each of its
/// adjacent faces, along with the index of the edge these share.
/// @see FaceEdgeIndex
using FaceAdjacencyMap = std::unordered_map<int, std::vector<FaceEdgeIndex>>;

/// Computes a mapping from each IndexFace index in @p faces to each of its
/// adjacent faces, along with the index of the edge these share.
///
/// For each face at index `i` in @p faces, a sequence to map each edge `j`
/// to its adjacent face and edge index is added to the map at `i`. If a given
/// edge is not adjacent to any face, an invalid face and edge index (both set
/// to -1, see FaceEdgeIndex) is put in its place.
/// @pre Any given pair of vertices is shared by two faces at most.
/// @warning If any of the preconditions is not met, this function will
///          abort execution.
FaceAdjacencyMap ComputeFaceAdjacencyMap(const std::vector<IndexFace>& faces);

/// Gets global position of the @p vertex in the given @p mesh.
/// @pre Given @p vertex belongs to the @p mesh.
/// @warning If any of the preconditions is not met, this function will
///          abort execution.
const math::Vector3& GetMeshFaceVertexPosition(const GeoMesh& mesh, const IndexFace::Vertex& vertex);

/// Gets normal vector of the @p vertex in the given @p mesh.
/// @pre Given @p vertex belongs to the @p mesh.
/// @warning If any of the preconditions is not met, this function will
///          abort execution.
const math::Vector3& GetMeshFaceVertexNormal(const GeoMesh& mesh, const IndexFace::Vertex& vertex);

/// Checks if all the IndexFace::Vertex instances, from @p first to
/// @p last, in the given @p mesh lie on the provided plane
/// defined by a normal non-zero vector @p n and a point @p p by
/// verifying all of them are within one @p tolerance distance, in
/// meters, away from it along the line subtended by its normal.
/// @pre Given @p vertices belong to the @p mesh.
/// @warning If any of the preconditions is not met, this function
///          will abort execution.
/// @tparam InputIt An IndexFace::Vertex container iterator type.
template <typename InputIt>
bool DoMeshVerticesLieOnPlane(const GeoMesh& mesh, InputIt first, InputIt last, const math::Vector3& n,
                              const math::Vector3& p, double tolerance) {
  return std::all_of(first, last, [&mesh, &n, &p, tolerance](const IndexFace::Vertex& vertex) {
    const math::Vector3& x_vertex = GetMeshFaceVertexPosition(mesh, vertex);
    const math::Vector3& n_vertex = GetMeshFaceVertexNormal(mesh, vertex);
    const double ctheta = std::abs(n.dot(n_vertex.normalized()));
    return (ctheta != 0. && DistanceToAPlane(n, p, x_vertex) / ctheta < tolerance);
  });
}

/// Checks if the @p face in the given @p mesh is coplanar with the
/// given plane defined by a normal non-zero vector @p n and a point @p p,
/// by verifying if all @p face vertices are within
/// one @p tolerance distance, in meters, from it.
/// @pre Given @p face belongs to the @p mesh.
/// @warning If any of the preconditions is not met, this function
///          will abort execution.
bool IsMeshFaceCoplanarWithPlane(const GeoMesh& mesh, const IndexFace& face, const math::Vector3& n,
                                 const math::Vector3& p, double tolerance);

/// Checks if the @p face in the given @p mesh is planar, by verifying
/// all @p face vertices lie on a plane using the given @p tolerance
/// (see DoMeshVerticesLieOnPlane()). Said plane, built out of the
/// first vertex position and normal in the @p face, is returned as
/// a plane defined by a point @p p and a normal non-zero vector @p n.
/// @param[in] mesh Is a `Inertial`-frame mesh.
/// @param[in] face Is a sequence of vertices with normals.
/// @param[in] tolerance Is the tolerance to compute face planarity.
/// @param[out] n Is a vector that with `p` define a plane that should contain all the face vertices.
/// @param[out] p Is a point that with `n` define a plane that should contain all the face vertices.
/// @pre Given @p n is not nullptr.
/// @pre Given @p p is not nullptr.
/// @pre Given @p face belongs to the @p mesh.
/// @pre Given @p face has at least three (3) vertices.
/// @warning If any of the preconditions is not met, this function
///          will abort execution.
bool IsMeshFacePlanar(const GeoMesh& mesh, const IndexFace& face, double tolerance, math::Vector3* n, math::Vector3* p);

/// Aggregates all coplanar faces adjacent to the referred face in the @p mesh.
/// @param mesh Mesh where faces are to be found.
/// @param start_face_index Index of the face to start aggregation with.
/// @param adjacent_faces_map Map of adjacent faces associated with the
///                           given @p mesh.
/// @param tolerance For coplanarity checks, in meters. See IsMeshFacePlanar()
///                  and IsMeshFaceCoplanarWithPlane() functions for further
///                  details.
/// @param visited_faces_indices The indices of the faces visited so far.
/// @returns The indices of the adjacent coplanar faces found.
/// @pre Given @p start_face_index is valid for the given @p mesh.
/// @pre Given @p tolerance is a positive real number.
/// @pre Given @p visited_faces_indices collection is not nullptr.
/// @pre Given @p start_face_index has not been visited yet
///      (i.e. visited_faces_indices.count(start_face_index) == 0).
/// @post All adjacent coplanar faces found are marked as visited.
/// @warning If any of the preconditions is not met, this function
///          will abort execution.
std::set<int> AggregateAdjacentCoplanarMeshFaces(const GeoMesh& mesh, int start_face_index,
                                                 const FaceAdjacencyMap& adjacent_faces_map, double tolerance,
                                                 std::set<int>* visited_faces_indices);

/// Finds the index to the first outer face edge in the given
/// @p simply_connected_faces_indices.
/// @param simply_connected_faces_indices Indices of the faces whose
///                                       outer face edge is to be found.
/// @param adjacent_faces_map Mapping of adjacent faces for the faces
///                           referred by @p simply_connected_faces_indices.
/// @returns The index of first outer face edge found or an invalid index
///          if it failed to find any due to unmet preconditions.
/// @pre The union of the all the faces referred by the given
///      @p simply_connected_faces_indices yields a simply
///      connected region (i.e. with no holes).
FaceEdgeIndex FindOuterFaceEdgeIndex(const std::set<int>& simply_connected_faces_indices,
                                     const FaceAdjacencyMap& adjacent_faces_map);

/// Index of a face vertex in a GeoMesh.
struct FaceVertexIndex {
  int face_index{-1};    ///< Index of the face the vertex belongs to, or
                         ///  -1 to mark index as invalid.
  int vertex_index{-1};  ///< Index of the face vertex, or -1 to mark index
                         /// as invalid.
};

/// Computes the contour of the simply connected region that all the faces
/// referred by the given @p simply_connected_faces_indices yield.
/// @param simply_connected_faces_indices Indices of the faces whose outer face
///                                       edge is to be found.
/// @param adjacent_faces_map Mapping of adjacent faces for the faces referred
///                           in @p simply_connected_faces_indices.
/// @returns Face vertices as a counter-clockwise contour.
/// @pre The union of the all the faces referred by the given
///      @p simply_connected_faces_indices yield a simply
///      connected region (i.e. with no holes).
std::vector<FaceVertexIndex> ComputeMeshFacesContour(const std::set<int>& simply_connected_faces_indices,
                                                     const FaceAdjacencyMap& adjacent_faces_map);

/// Gets the face vertex in the @p mesh referred by the given
/// @p face_vertex_index.
/// @pre Given @p face_vertex_index refers to a valid vertex
///      in the @p mesh.
/// @warning If any of the preconditions is not met, this function
///          will abort execution.
const IndexFace::Vertex& MeshFaceVertexAt(const GeoMesh& mesh, const FaceVertexIndex& face_vertex_index);

/// Applies the Douglas-Peucker simplification algorithm [1] over the
/// given collection of vertices.
///
/// - [1] Douglas, D. H., & Peucker, T. K. (1973). Algorithms for the
///       reduction of the number of points required to represent a
///       digitized line or its caricature. Cartographica: The
///       International Journal for Geographic Information and
///       Geovisualization, 10(2), 112–122.
/// @param first Iterator to first element of the collection.
/// @param last Iterator to the last element of the collection.
/// @param to_vertex A function to retrieve the vertex associated with
///                  an element of the collection (may be a pass-through).
/// @param to_edge A function to construct an edge out of a pair of
///                vertices. This function must return a std::pair containing
///                the origin vertex and the unit direction vector of the parametric line.
/// @param tolerance Simplification tolerance, in distance units.
/// @param output Output iterator for the simplification result.
/// @tparam InputIt Input iterators type.
/// @tparam VertexFn Vertex getter function type.
/// @tparam EdgeFn Edge building function type.
/// @tparam OutputIt  Output iterator type.
template <typename InputIt, typename VertexFn, typename EdgeFn, typename OutputIt>
void ApplyDouglasPeuckerSimplification(InputIt first, InputIt last, VertexFn to_vertex, EdgeFn to_edge,
                                       double tolerance, OutputIt output) {
  if (first == last) {
    *output++ = *last;
    return;
  }

  const std::pair<math::Vector3, math::Vector3> point_and_direction = to_edge(to_vertex(*first), to_vertex(*last));
  auto farthest = std::max_element(first, last, [&point_and_direction, &to_vertex](const auto& lhs, const auto& rhs) {
    return DistanceToALine(point_and_direction.first, point_and_direction.second, to_vertex(lhs)) <
           DistanceToALine(point_and_direction.first, point_and_direction.second, to_vertex(rhs));
  });
  if (DistanceToALine(point_and_direction.first, point_and_direction.second, to_vertex(*farthest)) > tolerance) {
    ApplyDouglasPeuckerSimplification(first, farthest, to_vertex, to_edge, tolerance, output);
    ApplyDouglasPeuckerSimplification(farthest + 1, last, to_vertex, to_edge, tolerance, output);
  } else {
    *output++ = *first;
    *output++ = *last;
  }
}

/// Simplifies the @p mesh faces' contour referred by the given
/// @p contour_indices by elimination of redundant vertices, i.e.
/// vertices that lie within one @p tolerance distance, in meters,
/// from the line that the their following and preceding vertices
/// subtend.
std::vector<FaceVertexIndex> SimplifyMeshFacesContour(const GeoMesh& mesh,
                                                      const std::vector<FaceVertexIndex>& contour_indices,
                                                      double tolerance);

/// Merges all the faces in the given @p mesh referred by
/// @p mergeable_faces_indices into a single GeoFace.
/// @param mesh Mesh the faces to be merged belong to.
/// @param mergeable_faces_indices Indices of the faces in
///                                @p mesh to be merged.
/// @param adjacent_faces_map Mapping of adjacent faces for the faces
///                           referred in @p mergeable_faces_indices.
/// @param tolerance For contour simplification, in meters. See
///                  SimplifyMeshFacesContour() function for further
///                  details.
/// @returns Merged mesh faces as a single GeoFace.
/// @pre The union of the all the faces referred by the given
///      @p mergeable_faces_indices yields a simply connected
///      region (i.e. with no holes).
GeoFace MergeMeshFaces(const GeoMesh& mesh, const std::set<int>& mergeable_faces_indices,
                       const FaceAdjacencyMap& adjacent_faces_map, double tolerance);

/// Simplifies a mesh by merging adjacent coplanar faces.
/// @param input_mesh Mesh to be simplified.
/// @param tolerance Tolerance for simplification, in meters. See
///                  MergeMeshFaces() and AggregateAdjacentCoplanarMeshFaces()
///                  functions for further details.
/// @returns Output, simplified mesh.
/// @pre The union of the all the faces in the given @p input_mesh
///      yields a simply connected region (i.e. with no holes).
GeoMesh SimplifyMeshFaces(const GeoMesh& input_mesh, double tolerance);

}  // namespace mesh
}  // namespace utility
}  // namespace maliput
