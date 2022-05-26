// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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

/// @file maliput_design.h
/// @page maliput_design Maliput Design
/// @author Matt Marjanović
/// @author Chien-Liang Fok
/// @author Agustin Alba Chicar
/// @date September 25, 2019
/// @tableofcontents
///
/// @section modeling_road_networks_for_simulation Modeling Road Networks for Simulation
///
/// This document describes `maliput`, a model of road networks for use in
/// agent and traffic simulations.  At the core of `maliput` is a
/// mathematical model of the geometry and topology of a road network.
/// That model is practically expressed by an abstract C++ API which is
/// intended to be independent of any specific on-disk format for
/// persistent road data.  Concrete implementations of the abstract API
/// allow various sources of road network data to be expressed via the
/// common `maliput` model.
///
/// @subsection objectives Objectives
///
/// Driving happens on roads (most of the time); the *road network* is a
/// fundamental structure in the task of driving.  Any non-trivial
/// simulation of driving will involve some model of roads --- the surface
/// on which the vehicles are moving.  Our top-level goals for the
/// `maliput` model are:
///
///  * make it easy to develop simple but rich agents to drive the
///    *ado*-cars, the secondary vehicles which interact with and exercise
///    the behaviors of the *ego*-cars; (1)
///
///  * provide a ground-truth for measuring the performance and behavior of
///    all agents in a simulation.
///
/// (1): The *ado* are the supporting actors in *Kyogen*, a form of Japanese
///      comic theater traditionally performed in the interludes between
///      Noh plays, featuring farcical depictions of daily life.
///
/// For both of these goals, we need to know where vehicles and other
/// objects are (and where they are going) with respect to one another *in
/// the context of the road*.  Thus, the `maliput` API provides methods to
/// answer questions such as "How close am I to the edge of the lane?" and
/// "What objects are within 100 meters ahead of me in my current lane?"
///
/// `maliput` is intended to be agnostic of the data source for a road network.
/// Concrete implementations for different data sources will expose the same
/// abstract interface.  Some networks may be completely synthetic (constructed
/// by hand, or even procedurally), others will be created from measurements
/// of real-life roads.
///
/// We expect to have implementations to support:
///  * assembling simple geometric primitives for procedural roads (e.g.,
///    completely synthetic roads for unit-test cases);
///  * interpolating smooth roads from geographic position and orientation
///    samples measured from real roads;
///  * loading of road networks stored in the [OpenDRIVE](http://opendrive.org)
///    format.
///
/// The C++ API is also intended to allow for tiling, i.e., instantiating
/// fragments of very large, complex road networks on-demand and disposing
/// of fragments that are no longer immediately necessary.
///
/// @section road_network_goemetry Road Network Geometry
///
/// @subsection geometry_model Geometry Model
///
/// @subsubsection overview Overview
///
/// At the core of `maliput` is a mathematical model for the geometry of
/// the space around a road network; it is a model of both the road
/// surface and the volume proximal to that surface.  In the
/// abstract, one can think of "a road" as a 2D manifold (the road
/// surface) embedded in 3D space (the physical universe).  The manifold
/// structure is important because much of driving involves figuring out
/// where things are in relation to the road.  The embedding is important,
/// too, because physical sensing and actuation (and realistic
/// visualization) happens in physical space.  In `maliput`, we consider the
/// *road volume* and not just the *road surface* because we want to be
/// able to describe objects and events which are not glued to the surface
/// --- e.g., stop lights, street signs, watermelons falling off of
/// trucks.
///
/// In super-mathy terms:
///  * The world containing the road network is approximated by an inertial,
///    locally (if not globally) flat, 3-dimensional Cartesian coordinate
///    system referred to as the *`Inertial`-frame*.
///  * The road surface is a bounded compact orientable 2-dimensional manifold
///    embedded in the @f$ \mathbb{R}^3 @f$ `Inertial`-frame via a @f$ G^1 @f$
///    continuous map from @f$ \mathbb{R}^2 \to \mathbb{R}^3 @f$.
///  * The road surface is extended via its normals into a bounded compact
///    orientable 3-dimensional road volume, also embedded in the
///    @f$ \mathbb{R}^3 @f$ `Inertial`-frame via a @f$ G^1 @f$ continuous map from
///    @f$ \mathbb{R}^3 \to \mathbb{R}^3 @f$.
///  * We impose the @f$ G^1 @f$ continuity constraint on all roads to ensure that there
///    is a consistent, well-defined orientation everywhere on the road's manifold.
///    (Abrupt changes in curvature are allowed, but cusps/kinks are not. Realistic
///    violations of this rule like potholes are modeled in a different layer.)
///    (Furthermore, we construct the maps over a finite partition of the
///     road volume, and over each partition, we require that the maps are
///     @f$ C^1 @f$ continuous.
///  * No continuity requirement is imposed on the portion of the world that is
///    not part of the road's manifold, including at the boundary between the road
///    and its surroundings. For example, the edge of a road may have a curb, which
///    can have a sudden elevation change.
///
/// In the lexicon of `maliput` and its C++ API, the road volume manifold is
/// called a `RoadGeometry`.  A `RoadGeometry` is partitioned into
/// `Segments`, which correspond to stretches of asphalt (and the space
/// above and/or below them).  Each `Segment` is a group of one or more
/// adjacent `Lanes`.  A `Lane` corresponds to a lane of travel on a road,
/// and defines a specific parameterization of the parent `Segment`'s
/// volume from a local *lane frame* into the `Inertial`-frame.  `Lanes` are
/// connected at `BranchPoints`, and the graph of `Lanes` and
/// `BranchPoints` describes the topology of a `RoadGeometry`. `Segments` which map
/// to intersecting volumes of the `Inertial`-frame (e.g., intersections) are grouped
/// together into `Junctions`.
///
/// In a sense, there are two complementary object graphs in `maliput`.
/// The container hierarchy (`Junctions` contain `Segments`, which contain
/// `Lanes`) groups together different views of the same regions of road
/// surface.  The routing graph (`Lanes` are joined end-to-end via
/// `BranchPoints`) describes how one can get from one region of the road
/// network to another.
///
/// A `RoadGeometry` may also model paths that are adjacent to roads like sidewalks.
/// If there is no @f$ G^1 @f$ continuity between the road and its adjacent paths, the two
/// must be separated by `Segment` boundaries. This is not in violation of Maliput's
/// continuity requirements because Maliput has no notion of laterally-adjacent
/// `Segments`.
///
/// @subsubsection scale_length_and_tolerances Scale Length and Tolerances
///
/// > TODO: Explain the concepts of linear tolerance, angular tolerance, and
/// > characteristic scale length.
///
/// @subsubsection inertial_frame_versus_lane_frame Inertial-frame versus Lane-frame
///
/// Two types of coordinate frames are used in this model: the (single)
/// `Inertial`-frame and the (multiple) `Lane`-frames.  In both, distances
/// are typically measured in units of meters.
///
/// The `Inertial`-frame is any right-handed 3D inertial Cartesian coordinate
/// system, with orthonormal basis @f$(\hat{x},\hat{y},\hat{z})@f$ and
/// positions expressed as triples @f$(x,y,z)@f$.  This could be a
/// globally-flat coordinate system, e.g., ECEF ("Earth-centered,
/// Earth-fixed").  Or, it could be a locally-flat projection of the
/// Earth's surface, e.g., a UTM ("Universal Transverse Mercator")
/// projection coupled with elevation.  No specific projection is mandated
/// by `maliput`. To disambiguate for one or another `Inertial`-frame choice, the
/// API uses the preferred `InertialPosition` type which can be interpreted for
/// one or another frame choice.
///
/// > *Currently:*  @f$\hat{z}@f$ is assumed to be *up*, with @f$z@f$ representing an
/// > altitude or elevation.  @f$\hat{x}@f$ and @f$\hat{y}@f$ span the horizontal
/// > plane.  Typically, the "ENU" convention is used: @f$\hat{x}@f$ points *East*
/// > and @f$\hat{y}@f$ points *North*.
///
/// Another frame will be defined, the `Backend`-frame which is
/// different from the `Inertial`-frame by an isometric transform. This frame is
/// also a right-handed 3D inertial Cartesian coordinate system with an
/// orthonormal basis. It exists and potentially differs from the
/// `Inertial`-frame because of differing contexts. Typically, the
/// `Inertial`-frame chosen matches that of the client, e.g., a
/// simulator that uses Maliput. Meanwhile, the `Backend`-frame typically
/// matches the underlying data used by a particular backend, e.g., an
/// OpenDRIVE file. The `Backend`-frame will use `Vector3` for coordinates to
/// properly differentiate from the `Inertial`-frame type, i.e. `InertialPosition`.
///
/// A `Lane`-frame is a right-handed orthonormal curvilinear coordinate system, with
/// positions expressed as coordinates @f$(s,r,h)@f$.  Each `Lane` in a `RoadGeometry`
/// defines its own embedding into the `Inertial` space, and thus each `Lane`
/// has its own `Lane`-frame.
///
/// When embedded into the `Inertial` space, @f$s@f$ represents longitudinal distance
/// (path-length) along a central reference curve (the *centerline*) which
/// defines a given `Lane`.  @f$r@f$ is lateral distance along the road surface,
/// the path length along a geodesic perpendicular to the centerline.
/// @f$h@f$ is height above the road surface, the distance along a normal.  Unless
/// the lane is completely straight and flat, a `Lane`-frame acts like a
/// non-inertial system: the @f$(s,r,h)@f$ are not isotropic (@f$s@f$ is only
/// guaranteed to correspond to true physical distance when @f$(r,h) =
/// (0,0)@f$ (i.e., along the centerline), and similarly @f$r@f$ only yields a
/// true physical distance when @f$h = 0@f$ (i.e., along the road surface).)
/// and the curves and twists in the embedding introduce fictitious
/// forces in equations of motion expressed in these coordinates.
///
/// > TODO: see discussion in https://github.com/ToyotaResearchInstitute/maliput_malidrive/issues/16
/// > For certain roads whose lateral profile is a straight line, geodetic curves
/// > would likely be straight lines. However, that's not true for the general
/// > case and the @f$r@f$ distance is not trivially computed. Most backend
/// > implementations might choose to approximate the arc length of the geodetic
/// > curve by a path length of a straight line measured in the `Inertial` space.
///
/// > TODO: Replace this gibberish with a proper description of the
/// > effects of the metric induced by the push forward of @f$W_L@f$.
/// > We also introduce the notion of /isotropic coordinates/
/// > @f$(\sigma,\rho,\eta)@f$ corresponding to the non-isotropic @f$(s,r,h)@f$.  At
/// > every point @f$(s,r,h)@f$ in a `Lane` with its local
/// > @f$(\hat{s},\hat{r},\hat{h})@f$ coordinate frame, we define a
/// > corresponding @f$(\hat{\sigma},\hat{\rho},\hat{\eta})@f$ frame with the
/// > same orientation but different scale factors which make it isotropic.
/// > We don't use @f$(\sigma,\rho,\eta)@f$ to parameterize the space of the
/// > `Lane`, but rather to talk about physically-relevant velocities and
/// > accelerations.  In other words, at a given point in a `Lane`, the
/// > magnitude of a velocity @f$(\dot{\sigma},\dot{\rho},\dot{\eta})@f$ is
/// > unchanged when mapped to @f$(\dot{x},\dot{y},\dot{z})@f$, and the
/// > direction undergoes the same rotation for all velocity vectors
/// > anchored to that point.
///
/// Finally, we will colloquially use the term "`Road`-frame" to refer to
/// a 4-tuple of parameters @f$(L,s,r,h)@f$ in which:
///  * @f$L@f$ is an identifier which uniquely names a `Lane`;
///  * @f$(s,r,h)@f$ are `Lane`-frame coordinates understood in the context
///    of `Lane` @f$L@f$.
///
/// One can construct a map @f$W: \lbrace(L,s,r,h)\rbrace \to \mathbb{R}^3@f$ from the
/// road manifold into the `Inertial` space, as a union of the per `Lane` maps.
/// This @f$W@f$ is technically an *immersion* and not an *embedding* because
/// it is not necessarily 1-to-1; as described later on, multiple `Lanes`
/// in the same `Segment` will double-cover the same region of the
/// @f$\mathbb{R}^3@f$ `Inertial`-frame.  Also, due to our representation of
/// routing, double-coverage will occur where streets cross to form
/// intersections, or where highways split or merge.  This needs to be
/// considered when determining the possible interactions of agents or
/// objects that are located in nominally distinct regions of the `Lane` network.
///
/// > Note:  Due to certain geometric constraints in `Lane`-frame parameterization,
/// > some regions of the `RoadGeometry` manifold may not be covered by the
/// > `Lane`-frame of any `Lane`.  We anticipate needing an additional set of
/// > surface/volume parameterizations in the future to complete the picture.
///
/// @subsubsection lanes_as_lanes Lanes as Lanes
///
/// A `Lane` represents a lane of travel in a road network, expressing a path
/// along a stretch of asphalt as well as a parameterization of that asphalt
/// from one lateral edge to the other (including adjacent lanes of travel,
/// shoulders, etc).
///
/// As discussed above, a `Lane`, identified by @f$L@f$, defines a map @f$W_L@f$
/// from curvilinear coordinates to the `Inertial`-frame:
///
/// @f[
/// W_L: (s,r,h) \mapsto (x,y,z), \text{ for } s \in [0, s_\text{max}]
/// @f]
///
/// The curve traced out by @f$W_L@f$ along longitudinal coordinate @f$s@f$ (while @f$r@f$
/// and @f$h@f$ are fixed to zero) is called the *centerline* of the `Lane`:
///
/// @f[
/// C_L: s \mapsto (x,y,z), = W_L(s,0,0) \text{ for } s \in [0, s_\text{max}]
/// @f]
///
/// The centerline is nominally the ideal trajectory of a vehicle traveling
/// in the lane (and it is not necessarily in the geometric center of the lane,
/// despite the name).  @f$W_L@f$ is required to be @f$C^1@f$ continuous, and thus
/// @f$C_L@f$ is also required to be @f$C^1@f$ continuous.
///
/// The space of the `Lane` is bounded in @f$s@f$ by @f$s \in [0,
/// s_\text{max}]@f$.  @f$s_\text{max}@f$ is called the *length* of the `Lane`
/// and is in fact the path-length of the centerline @f$C_L@f$ (in both the
/// `Lane`-frame and the `Inertial`-frame).  The @f$s=0@f$ end of a `Lane` is
/// labeled the *start end*, and the @f$s=s_\text{max}@f$ end is the *finish
/// end*.  However, a `Lane` is just a stretch of pavement with no
/// preferred travel direction, and there is no direction of travel
/// implied by these labels. (2)
///
/// (2): Travel restrictions on a `Lane` are indicated by road rule annotations,
/// described later in section @sa RoadRulebook.
///
/// A `Lane` is bounded laterally by *segment bounds*, @f$r \in B_\text{segment}(s)@f$, where
///
/// @f[
/// B_\text{segment}: s \mapsto [r_\text{min}, r_\text{max}] \text{ s.t. } r_\text{min}<=0 \text{ and } r_\text{max}>=0
/// @f]
///
/// defines inclusive min/max bounds which depend only on @f$s@f$.  These are
/// the @f$segment bounds@f$ for the `Lane`, the valid domain of @f$r@f$, which
/// is intended to represent the full lateral extent of the `Segment` including
/// all adjacent `Lanes`.
///
/// > TODO: This begs for a picture.
///
/// A `Lane` is also characterized by *nominal bounds*, @f$r \in B_\text{nominal}(s)@f$, where
///
/// @f[
/// B_\text{nominal}: s \mapsto [r_\text{min}, r_\text{max}] \text{ s.t. } B_\text{nominal} \subseteq B_\text{segment}
/// @f]
///
/// which indicate what is considered to be "in" that specific travel lane
/// (e.g., between the stripes).
///
/// A `Lane` is bounded in height by @f$h \in H_\text{lane}(s,r)@f$, where
///
/// @f[
/// H_\text{lane}: (s,r) \mapsto [h_\text{min}, h_\text{max}] \text{ s.t. } h_\text{min}<=0 \text{ and } h_\text{max}>=0
/// @f]
///
/// defines inclusive min/max bounds which depend on @f$s@f$ and @f$r@f$.  These define
/// the valid domain of @f$h@f$, which represents the full extent of the volume
/// (above and possibly below the road surface) modeled by the `Lane`.
/// Typically, @f$h_\text{min}@f$ is zero, but having @f$h_\text{min}<0@f$ allows a
/// `Lane` to describe the location of subterranean features (e.g., measurements
/// made by ground-penetrating radar).
///
/// > Note: Because of the orthogonality of the @f$(s,r,h)@f$ coordinates, a
/// > curve with constant non-zero @f$(r,h)@f$ (imagine @f$r@f$ and @f$h@f$ "grid
/// > lines") is basically a parallel curve to the centerline @f$C_L@f$.  Thus,
/// > the shape of @f$C_L@f$ and/or the road surface may
/// > produce limits to @f$(r,h)@f$ before such a curve develops a cusp.
/// > The current definitions of @f$B_\text{segment}@f$ and
/// > @f$H_\text{lane}@f$ conflate the bounds of the /segment/ volume
/// > (e.g., pavement and free space under bridges) with the bounds of the
/// > /modeled/ volume (e.g., the bounds on @f$r@f$ and @f$h@f$ which maintain
/// > @f$G^1@f$ continuity, avoiding cusps).  Hence, the road surface may continue
/// > into regions that cannot be properly represented by the parameterization
/// > of a given `Lane`.
///
/// @subsubsection lanes_joined_via_branchpoints Lanes Joined End-to-End via BranchPoints
///
/// `BranchPoints` are the points where `Lanes` are connected end-to-end.
/// They are so named because they are the branch-points in the decision
/// tree of an agent driving in the network, which must decide which
/// new `Lane` to follow at the end of its current `Lane`.  Each end
/// (*start* or *finish*) of a `Lane` has an associated
/// `BranchPoint` (3).  Each `BranchPoint` has at least one `Lane` associated
/// with it, typically two, and often more than that (when `Lanes` merge/diverge). (4)
///
/// We only allow `BranchPoints` to occur at the ends of `Lanes`, specifically at
/// the ends of their centerlines (@f$C_L(s)@f$). We also require that the centerlines
/// of the `Lanes` joined at a `BranchPoint` are @f$G^1@f$ continuous.  Together with
/// the earlier-stated requirement of overall @f$G^1@f$ continuity of the road surface
/// and the conditions on @f$r@f$ and @f$h@f$ being path-lengths, this implies that:
///  1. The location of a `BranchPoint` is a well-defined point in the `Inertial`-frame.
///  2. The tangent vectors of the @f$C_L@f$ curves are either parallel or
///     antiparallel with each other at the
///     `BranchPoint`.  In fact, except for the signs of @f$\hat{s}@f$ and @f$\hat{r}@f$,
///     the frames of all the `Lanes` will have the same orientation and scale.
///  3. Given two `Lanes` @f$J@f$ and @f$K@f$ joined at a `BranchPoint` located at
///     the *finish* end of @f$J@f$, then a position @f$(s_\text{max,J}, r, h)_J@f$
///     in @f$J@f$ will map to either @f$(0, r, h)@f$ or @f$(s_\text{max,K}, -r,
///     h)_K@f$ in @f$K@f$ (depending on which end of @f$K@f$ is at the `BranchPoint`).
///
/// Given point (2) above, one can imagine multiple `Lanes` converging on one
/// side of a `BranchPoint`, flowing smoothly through it, and diverging into
/// other `Lanes` on the other side.  If one considers the
/// "outward-traveling tangent vector" of each `Lane`, then the `Lanes` can be grouped by
/// common orientation of outward-traveling tangent vector into at most two
/// groups.  Thus, a `BranchPoint` fundamentally has two sides to it.  The
/// sides are arbitrary, so we label them with the arbitrary names "A" and
/// "B".  With respect to a specific `Lane` @f$J@f$, regardless of which side @f$J@f$
/// is on (be it A or B):
///  * the `Lanes` on the "same side" as @f$J@f$ are the *confluent lanes* of @f$J@f$;
///  * the `Lanes` on the "other side" are the *ongoing lanes* of @f$J@f$.
///
/// > TODO: figure with sample branch-point topologies:
/// > * 1:1 --- simple continuation of one lane onto another;
/// > * 1:2 --- a split of one lane to two;
/// > * 1:3 --- a split of one to three, e.g., paths through an intersection
/// >   with left and right turns available;
/// > * 2:2 --- a merge/split, e.g., entering and/or exiting a roundabout;
/// > * 1:0 --- you've reached the end of the road, my friend.
///
/// A `BranchPoint` bears one additional element of information.  For each
/// `Lane`, one of its ongoing `Lanes` may optionally be named as its
/// *default-branch*.  This serves as a semantic hint about the structure
/// of the road.  The default-branch represents the notion of "which
/// branch should I choose in order to continue straight ahead" (5).  For
/// example, when entering a 4-way intersection, a `Lane` may terminate
/// with three ongoing branches: turning left, going straight, and turning
/// right; the "go straight" branch would be designated the
/// default-branch.  Likewise, at a split in a highway, one fork might
/// be considered the same highway, whereas the other is considered an exit.
/// (Also, note that default-branch relationships between `Lanes` need not
/// be symmetric.)
///
/// (3): This means a `Lane` has precisely two
/// `BranchPoints`, except for the peculiar case of a `Lane` which loops
/// around and connects to itself, at a single `BranchPoint`.
///
/// (4): `BranchPoint` with only a single `Lane` attached to it is basically a
/// dead-end.
///
/// (5): At the *finish* end of a `Lane`, this is just the tangent of @f$C_L@f$;
/// at the *start* end of a `Lane`, it's the negative of the tangent, pointing
/// in the @f$-s@f$ direction instead of the @f$+s@f$ direction.
///
/// @subsubsection adjacent_lanes_grouped_into_segments Adjacent Lanes Grouped into Segments
///
/// In real roads, the pavement is often divided into multiple adjacent
/// lanes of travel; in `maliput`, adjacent `Lanes` are grouped together
/// into `Segments`.  The basic idea is that a `Segment` corresponds to a
/// longitudinal stretch of pavement, and each `Lane` in that `Segment`
/// presents a different @f$(s,r,h)@f$ parameterization of that same pavement.
///
/// We would like for the segment-bounds of each `Lane` to map to the
/// same extent of physical space in the `Inertial`-frame, but that isn't always
/// possible due to the geometric constraints of parallel curves.  However,
/// we do require that the union of the segment-bounds of all `Lanes`
/// in a `Segment` is simply-connected.  This means that:
///  * a `Segment` doesn't have any "holes" in its segment space (e.g.,
///    no impassable monument in the middle of the road);
///  * it is always possible to drive from a position in one `Lane`-frame
///    to a position in another `Lane`-frame, though it may require
///    expressing intermediate steps in other `Lanes` to do it.
///
/// Within a `Segment`, we only allow the intersection of two `Lane` centerlines
/// (such as a lane merge/split) to occur at the endpoints of the `Lanes`,
/// which further implies that it may occur only at a `BranchPoint`.
/// This allows us to impose another constraint
/// on `Lanes` in a `Segment`:  they must be oriented and shaped such
/// that there is a consistent "right-to-left" ordering in terms of
/// increasing @f$r@f$.  In other words, within a `Segment`:
///  1. A `Lane` @f$K@f$ is considered "left of" `Lane` @f$J@f$ if and only if
///     there exists a point on the centerline @f$C_{K}@f$ of @f$K@f$ that has a
///     position with @f$r > 0@f$ in the `Lane`-frame of @f$J@f$.  @f$K@f$ is "right
///     of" @f$J@f$ if and only if a point exists on @f$C_{K}@f$ with position
///     @f$r < 0@f$ in the frame of @f$J@f$.
///  2. If and only if `Lane` @f$K@f$ is to the left of `Lane` @f$J@f$, then @f$J@f$ must
///     be to the right of @f$K@f$.
///  3. For every pair of distinct `Lanes` @f$J@f$ and @f$K@f$, @f$K@f$ must be either
///     to the left or to the right of @f$J@f$, and may not be both.
/// A consequence of this ((2) in particular) is that the /start/ and /finish/
/// ends of all the `Lanes` in a `Segment` are grouped together respectively
/// so that the `Lanes` are generally "pointing in the same direction".
/// Given the consistent ordering, we index the `Lanes` in a `Segment`
/// with unique integers, beginning with zero for the rightmost `Lane` and
/// increasing leftward.
///
/// @subsubsection intersecting_segments_grouped_into_junctions Intersecting Segments Grouped into Junction
///
/// It is possible for multiple `Segments` to cover the same pavement.
/// In fact, that is how intersections are represented, by criss-crossing
/// `Segments` which define the different paths through an intersection.
/// Overlapping `Segments` also occur where the road merges or diverges,
/// such as on-ramps, exit ramps, traffic circles, and a road that splits
/// to go around an impassable monument.
///
/// `Segments` which map to intersecting volumes in the `Inertial`-frame (in
/// terms of the union of the segment-bounds of their `Lanes`) are
/// grouped together into a `Junction`.  The primary (sole?) purpose of a
/// `Junction` is to indicate that objects in its component `Segments` may
/// spatially interact with each other (e.g., collide!).  Conversely, if
/// two `Segments` belong to two distinct `Junction`, then objects within
/// their respective segment-bounds should /not/ be touching.  (Note
/// that in considering intersection, we ignore the overlaps
/// that may occur where `Segments` join end-to-end via their `Lanes`.)
///
/// Every `Segment` must belong to one and only one `Junction`, and
/// every `Junction` must contain at least one `Segment`.
///
/// When designing/implementing a `RoadGeometry`, it is good practice to
/// structure the `Segments` to minimize the spatial extent of
/// `Junction`.  For example, a single long `Segment` which crosses
/// through two intersections would cause both intersections to belong
/// to the same `Junction`.  It would be better to split that single `Segment`
/// into three:  one crossing each intersection and one in-between that
/// joins those two end-to-end, resulting in three independent `Junction`
/// that are better localized.
///
/// @subsection abstract_geometry_api_roadgeometry Abstract Geometry API: RoadGeometry
///
/// > TODO:  Explain semantics of object ID's.  (cross-referencing, tiling,
/// > debugging, visualization)
///
/// > TODO:  Reference to `maliput::api` doxygen.
///
/// @subsubsection basic_types Basic Types
///
/// * `InertialPosition`
/// * `LanePosition`
/// * `RoadPosition`
/// * ...
///
/// @subsubsection roadgeometry RoadGeometry
///
/// * accessors for component `Junctions`
/// * accessors for component `BranchPoints`
/// * accessors for characteristic lengths and tolerances
///   * `linear_tolerance`
///   * `angular_tolerance`
///   * `scale_length`
///
/// @subsubsection junction Junction
///
/// * accessors for parent `RoadGeometry`, component `Junctions`
///
/// @subsubsection segment Segment
///
/// * accessors for parent `Junction`, component `Lanes`
///
/// @subsubsection lane Lane
///
/// * nominal @f$r@f$ bounds,
///   @f$B_\text{nominal}: (s) \mapsto [r_\text{min}, r_\text{max}]@f$
/// * segment @f$r@f$ bounds,
///   @f$B_\text{segment}: (s) \mapsto [r_\text{min}, r_\text{max}]@f$
/// * segment @f$h@f$ bounds,
///   @f$H_\text{lane}: (s,r) \mapsto [h_\text{min}, h_\text{max}]@f$
/// * embedding @f$W_L: (s,r,h) \mapsto (x,y,z)@f$
/// * inverse @f$W_L^{-1}: (x,y,z) \mapsto (s,r,h)@f$
/// * `Lane`-frame orientation
///   @f$Q: (s,r,h) \mapsto \text{orientation of }(\hat{s},\hat{r},\hat{h})@f$
/// * isotropic scale factors
///   @f$S: (s,r,h) \mapsto (\frac{ds}{d\sigma},\frac{dr}{d\rho},\frac{dh}{d\eta})@f$
/// * derivatives of @f$W_L@f$ (to compute fictitious forces)
/// * accessors for parent `Segment`, associated `BranchPoints`,
///   and left/right `Lanes`, to traverse the object graph.
///
/// @subsubsection branchpoints BranchPoints
///
/// * accessors for `Lanes` on each side ("A" versus "B")
/// * accessor for the set of confluent `Lanes` for a given `Lane`
/// * accessor for the set of ongoing `Lanes` for a given `Lane`
/// * accessor for the default branch (ongoing `Lane`) for a given `Lane`
/// * accessor for parent `RoadGeometry`
///
/// @section rules_and_features_databases Rules and Features Databases
///
/// @subsection rules_of_the_road Rules of the Road: RoadRulebook
///
/// A `RoadRulebook` (see @ref road-rulebook-outline_img "figure" ) expresses the semantic
/// "rules of the road" for a road network, as rule elements associated to
/// components of a `RoadGeometry`.  In a real, physical road network, road
/// rules are typically signaled to users via signs or striping, though
/// some rules are expected to be prior knowledge (e.g., "We drive on the
/// right-hand side here.").  `RoadRulebook` abstracts away from both the
/// physical artifacts and the symbolic state of such signals, and directly
/// represents the intended use of a road network at a semantic level.
///
/// We define three levels of knowledge of rules of the road:
///  * *Physical* *Sensory* comprises the physical artifacts (or simulated model
///    thereof) which signal rules to the sensors of humans or vehicles.
///    E.g., a traffic light of certain design hanging above a road,
///    emitting light; a white / black metal sign with numbers and words,
///    posted next to the road; a sequence of short yellow stripes painted
///    on the ground.
///  * *Symbolic* is the discrete state of the signals, abstracted away from
///    the specifics of the physical manifestation.  E.g., a traffic light
///    with four bulbs, of which the red one and the green left-facing
///    arrow are illuminated; a speed limit sign bearing a limit of 45
///    miles per hour; a dashed-yellow lane separation line.
///  * *Semantic* is the intended rules of the road, whether from implicit
///    knowledge, or conveyed via symbols and signals.  E.g., cars
///    traveling forward through the intersection must stop, but
///    left-turning cars may proceed; the speed limit for a specific
///    stretch of road is 45 mph; lane-change to the left in order to pass
///    is permitted.
///
/// The `RoadRulebook` interface only concerns the semantic level, which
/// is the level required to provide oracular /ado/ cars with interesting
/// interactive behaviors. (Future API's may be developed to express
/// the sensory and symbolic levels of expression, and to coordinate
/// between all three as required.)
///
/// @anchor road-rulebook-outline_img
/// @image html road-rulebook-outline.svg "`RoadRulebook` outline."
///
/// There are two rule APIs. One that is based on static types which is
/// deprecated, and the other one which relies on just two C++ distinct types
/// but allows further customization and extension by API users. In the
/// following sections, the two APIs are described.
///
/// @subsubsection new_rules_types New Rule API description
///
/// There are three rule C++ types: `Rule`, `DiscreteValueRule` and
/// `RangeValueRule`. The parent type, `Rule`, helps to hold the basic
/// information and semantic relationships with other rules, and other entities
/// in the world, like light bulbs. `DiscreteValeRule` is a rule type thought
/// for discrete states, comprising from string based representations to numeric
/// ones. On the other hand, `RangeValueRule` completes the universe of rule
/// types by supporting numeric ranges in which certain magnitudes may vary.
///
/// Each rule instance may have one or multiple states which are `Rule::State`
/// and its customized implementations `DiscreteValueRule::DiscreteValue` and
/// `RangeValueRule::Range`. These states are defined by common attributes like
/// other related rules (for increased semantics and hierarchies), severity
/// levels (to moderate the action of agents), related unique IDs (thought for
/// traffic light bulb identification with the rule state itself) and the value
/// they hold (either a string representation or a numeric range).
///
/// Rule state dynamics, i.e. the change of `DiscreteValueRule::DiscreteValue`
/// and `RangeValueRule::Range` to another one for each rule is controlled by
/// state providers. Only interfaces for `DiscreteValueRuleStateProvider` and
/// `RangeValueRuleStateProvider` are defined, meaning that custom
/// implementations are delegated to designers so as to control state transition
/// behaviors in the context of a simulation. These providers include the notion
/// of time. This is the only place where it appears due to the time agnostic
/// nature of the vast majority of the API. "Static" rules are those with a
/// unique configured state, but there is no direct API provisioning of that
/// information.
///
/// Finally, each rule instance is identified by a unique ID and multiple rules
/// may share the same rule type. The rule ID is backed by `Rule::Id` and
/// the rule type ID is backed by `Rule::TypeId`. There rule type itself under
/// this framework consists of:
///  * The rule type ID.
///  * All possible rule states (either `DiscreteValueRule::DiscreteValue` or
///    `RangeValueRule::Range`).
///
/// Given that two completely unrelated semantic rule types like the
/// "Right-Of-Way" and "Direction-Usage" are represented by the same C++ type,
/// `DiscreteValueRule`. The possible rule states for a "Right-Of-Way" rule type
/// and "Direction-Usage" rule type are stored in a `RuleRegistry`. The
/// `RuleRegistry` safely constructs rules with types that are defined at
/// runtime or previously in a separate location or store. At runtime, agents or
/// generically speaking API consumers might query the possible rule states for
/// a given rule type as well as construct rules to load into a `RoadRulebook`
/// implementation.
///
/// @paragraph new_rule_common_types Common rule types and their implementations
///
/// The majority of road networks present certain rule types that are common in
/// terms of their semantic attributes, not necessarily in their relationships
/// or state phasing. We can identify:
///
///  * Speed limits - define speed limits for agents - `RangeValueRule`
///  * Right of way - control of right-of-way / priority on specific routes -
///    `DiscreteValueRule`
///  * Direction usage - define the direction of travel for agents -
///    `DiscreteValueRule`
///  * Vehicle stop in zone behavior - define whether agents can stop and for
///    how long - `DiscreteValueRule`.
///
/// These types are already provided with base implementations so consumers can
/// quickly build up a `RuleRegistry` from them. It is opt-in, which proves to
/// be flexible enough for most use cases. Backend implementations might define
/// their own types atop of these ones or just replace them with their own
/// rule states and types.
///
/// @subsubsection common_region_entities Common Region Entities
///
/// A few common entities, which identify regions of the road network, occur in
/// the various rule types:
///
///  * `LaneId`: unique ID of a `Lane` in a `RoadGeometry`;
///  * `SRange`: inclusive longitudinal range @f$[s_0, s_1]@f$ between two
///    s-coordinates;
///  * `LaneSRange`: a `LaneId` paired with an `SRange`, describing a longitudinal
///    range of a specific `Lane`;
///  * `LaneSRoute`: a sequence of `LaneSRange`'s which describe a contiguous
///    longitudinal path that may span multiple end-to-end connected `Lane`'s;
///  * `LaneIdEnd`: a pair of `LaneId` and an "end" specifier, which describes
///    either the start or finish of a specific `Lane`.
///
/// > Note: regions are attributes of new and old rule descriptions. They define
/// > the applicability space of each rule.
///
/// @subsubsection roadrulebook_queries Queries to the `RoadRulebook`
///
/// The `RoadRulebook` API allows to query the collection of rules by:
///  * No restriction: can retrieve all rules.
///  * Rule ID: which are guaranteed to be unique at rule construction.
///  * By region: whether rule's applicability in the `RoadGeometry` intersects
///    the query region.
///
/// These queries are convenient because of the properties the rules themselves
/// have and what is interesting to a simulated agent. One important aspect of
/// the rules are their state dynamics which are handled in a different book
/// though.
///
/// @subsubsection deprecated_rule_api [DEPRECATED] Old rule API
///
/// We distinguish two kinds of state:
///  * *Static state* comprises the aspects of a simulation which are
///    established before the simulation begins and which cannot evolve
///    during the runtime of the simulation.  This can be considered to be
///    the configuration of a simulation.
///  * *Dynamic state* comprises the aspects of a simulation which can evolve
///    during the runtime as the simulation's time progresses.
///
/// The `RoadRulebook` design decouples static state from dynamic
/// state. Dynamic state needs to be managed during the runtime of a
/// simulation, and different simulation frameworks have different
/// requirements for how they store and manage dynamic state.  In
/// particular, the `drake` system framework requires that all dynamic state
/// can be externalized and collated into a single generic state vector
/// (called the “Context”), and the `RoadRulebook` design facilitates such a
/// scheme. Decoupling the dynamic and static state also aids development;
/// once the (small) interface between the two is established, development
/// of API’s for each kind of state can proceed in parallel.
///
/// `RoadRulebook` is an abstract interface which provides query methods to
/// return rule instances which match some filter parameters, e.g., rules
/// which involve a specified `Lane`.  Each flavor of rule is represented by
/// a different `*Rule` class.  Rules are associated to a road network by
/// referring to components of a `RoadGeometry` via component ID’s. Each
/// rule is itself identified by a unique type-specific ID.  This ID is the
/// handle for manipulating the rule during rulebook configuration, and for
/// associating the rule with physical / symbolic models and/or dynamic state
/// in a simulation.  A rule generally consists of static state, e.g., the
/// speed limit as posted for a lane. Some rules may involve dynamic state
/// as well. Any dynamic state will be provided by a separate entity, with
/// an abstract interface for each flavor of dynamic state. For example, a
/// `RightOfWayRule` may refer to dynamic state (e.g., if it represents a
/// traffic light) via its `RightOfWayRule::Id`. An implementation of the
/// `RightOfWayStateProvider` abstract interface will, via its `GetState()`
/// method, return the current state for a given `RightOfWayRule::Id`.  How
/// those states are managed and evolved over time is up to the
/// implementation.
///
/// Road rules can generally be interpreted as restrictions on behavior,
/// and absent any rules, behavior is unrestricted (by rules of the road).
/// For example, if a `RoadRulebook` does not provide a `SpeedLimitRule`
/// for some section of the road network, then there is no speed limit
/// established for that section of road.  Whether or not an agent follows
/// the rules is up to the agent; `RoadRulebook` merely provides the rules.
///
/// Six rule types are currently defined or proposed:
///
///  * `SpeedLimitRule` - speed limits
///  * `RightOfWayRule` - control of right-of-way / priority on specific routes
///  * TODO: `DirectionUsageRule` - direction-of-travel specification
///  * TODO: `LaneChangeRule` - adjacent-lane transition restrictions
///  * TODO: `OngoingRouteRule` - turning restrictions
///  * TODO: `PreferentialUseRule` - lane-based vehicle-type restrictions (e.g.,
///    HOV lanes)
///
/// @subsubsection speed_limit_rules SpeedLimitRule: Speed Limits
///
/// A `SpeedLimitRule` describes speed limits on a longitudinal range of a Lane.
/// It comprises:
///
///  * id
///  * zone (`LaneSRange`)
///  * maximum and minimum speed limits (in which a minimum of zero is
///    effectively no minimum)
///  * severity:
///    * *strict* (e.g., in the US, black-on-white posted limit)
///    * *advisory* (e.g., in the US, black-on-yellow advisory limit on curves)
///    * TODO: applicable vehicle type (for limits applying to specific types):
///      * any
///      * trucks
///      * ...
///    * TODO: time-of-day/calendar condition
///
/// @subsubsection right_of_way_rule RightOfWayRule: Stopping and Yielding
///
/// `RightOfWayRule` describes which vehicles have right-of-way (also
/// known as "priority"). Note that "right of way" does not mean "right
/// to smash through obstacles".  A green light means
/// that other cars should not enter an intersection, but the light turning
/// green will not magically clear an intersection.  Even after acquiring
/// the right-of-way, a vehicle should still respect the physical reality
/// of its environment and operate in a safe manner. When operating on
/// intersecting regions of the road network.  In the real world, such
/// rules are typically signaled by stop signs, yield signs, and traffic
/// lights, or are understood as implicit knowledge of the local laws
/// (e.g., "vehicle on the right has priority at uncontrolled
/// intersections").
///
/// A `RightOfWayRule` instance is a collection of `RightOfWayRule::State`
/// elements which all describe the right-of-way rules pertaining to a
/// specific `zone` in the road network.  The elements of a `RightOfWayRule` are:
///
/// <table>
///   <tr><td>`id`          <td>unique `RightOfWayRule::Id`
///   <tr><td>`zone`        <td>`LaneSRoute`
///   <tr><td>`zone_type`   <td>`ZoneType enum {StopExcluded, StopAllowed}`
///   <tr><td>`states`      <td>set of `State` mapped by `State::Id`
/// </table>
///
/// The `zone` is a directed longitudinal path in the road network,
/// represented as a `LaneSRoute`; the rule applies to any vehicle
/// traversing forward through the `zone`.  The `zone_type` specifies
/// whether or not vehicles are allowed to come to a stop within the
/// `zone`.  If the type is `StopExcluded`, then vehicles should not
/// enter the `zone` if they do not expect to be able to completely
/// transit the `zone` while they have the right-of-way, and vehicles
/// should continue to transit and exit the `zone` if they lose the
/// right-of-way while in the `zone`.  `StopExcluded` implies a
/// "stop line" at the beginning of the `zone`.  `StopAllowed` has
/// none of these expectations or restrictions.
///
/// Each `State` comprises:
///
/// <table>
///   <tr><td>`id`          <td>`State::Id` (unique within the context of the rule instance)
///   <tr><td>`type`        <td>`State::Type enum: {Go, Stop, StopThenGo}`
///   <tr><td>`yield_to`    <td>list of `RightOfWayRule::Id`
/// </table>
///
/// The state's `type` indicates whether a vehicle can *Go* (has
/// right-of-way), must *Stop* (does not have right-of-way), or must
/// *StopThenGo* (has right-of-way after coming to a complete stop).
/// The *Go* and *StopThenGo* types are modulated by `yield_to`, which is
/// a (possibly empty) list of references to other rule instances
/// whose right-of-way supersedes this rule.  A vehicle subject to a
/// non-empty `yield_to` list does not necessarily have to stop, but its
/// behavior should not hamper or interfere with the motion of
/// vehicles which are controlled by rules in the `yield_to` list.
///
/// Only one `State` of a rule may be in effect at any given time.  A rule
/// instance which defines only a single `State` is called a *static
/// rule*; its meaning is entirely static and fixed for all time.
/// Conversely, a right-of-way rule instance with multiple `State`
/// elements is a *dynamic rule*.  Although the collection of possible
/// `State`'s of a dynamic rule are fixed and described by the rule
/// instance, knowing which `State` is in effect at any given time
/// requires querying a `RightOfWayStateProvider`.
///
/// `RightOfWayStateProvider` is an abstract interface that provides a query
/// method that accepts a `RightOfWayRule::Id` and returns a result containing:
///
/// <table>
///   <tr><td>`current_id`          <td>`State::Id`
///   <tr><td>`next_id`             <td>std::optional `State::Id`
///   <tr><td>`next.duration_until` <td>std::optional `double`
/// </table>
///
/// `current_id` is the current `State` of the rule.  `next_id` is the
/// *next* `State` of the rule, if a transition is anticipated and the next
/// state is known.  `next.duration_until` is the duration, if known,
/// until the transition to the known next state.
///
/// Following are discussions on `RightOfWayRule` configurations
/// for a few example scenarios.
///
/// *Example: Uncontrolled Midblock Pedestrian Crosswalk*
///
/// @anchor RoWR-lone-crosswalk
/// @image html right-of-way-example-lone-crosswalk.svg "Uncontrolled midblock pedestrian crosswalk."
///
/// @ref RoWR-lone-crosswalk "Figure" illustrates a very simple scenario:
///
///   * One-way traffic flows northbound, crossed by an uncontrolled pedestrian
///     crosswalk at midblock.
///   * The pedestrian traffic route is not modeled in the road network, so only
///     one zone (for the vehicular traffic intersecting the crosswalk) is involved.
///
/// With only one zone and no changing signals, a single, static
/// `RightOfWayRule` is required:
///
/// <table>
///   <tr><th>Rule + Zone <th>`zone_type`     <th>State `id`  <th>`type`  <th>`yield_to`
///   <tr><td>"North"     <td>*StopExcluded*  <td>"static"    <td>*Go*    <td>---
/// </table>
///
/// The `State::Id` chosen here ("static") is arbitrary.
///
/// The zone is a `LaneSRoute` spanning from the southern edge of the
/// crosswalk to the northern edge,
/// with zone-type *StopExcluded*, which means that stopping
/// within the zone is not allowed.  The single state has type *Go*, which
/// means that vehicles have the right-of-way to proceed.  (Note that
/// "when it is safe to do so" is always implied with any rule.)
/// Furthermore, that single state has an empty `yield_to` list, which
/// means no intersecting paths have priority over this one. (In fact,
/// there are no intersecting paths.)
///
/// This is a pretty trivial rule, since it has a single state which is
/// always "Go".  However, it serves to capture the requirement that
/// when a vehicle *does* stop, it should avoid stopping in the crosswalk.
///
/// Note that a more complete scenario, which actually modeled pedestrian
/// traffic, would likely represent the crosswalk as a lane of its own
/// (intersecting the vehicular lane) and the "North" rule would specify
/// yielding to that crosswalk lane via the `yield_to` element.
///
/// *Example: One-way Side Street onto Two-Lane Artery*
///
/// @anchor RoWR-one-way-to-two-way
/// @image html right-of-way-example-one-way-side-street.svg "Intersection with one-way side street onto two-lane
/// artery."
///
/// @ref RoWR-one-way-to-two-way "Figure" is a scenario with an intersection:
///
///   * East-west traffic is two way and uncontrolled.
///   * Northbound traffic is controlled by a stop sign.
///   * There are four zones (paths) traversing the intersection
///     (illustrated by the four arrows).
///
/// With four zones and no changing signals, four static rules are
/// required.  The rules have been labeled by a combination of the initial
/// heading and the turn direction of their paths. (E.g., "NB/Left" refers
/// to "the northbound path that turns left".)  All the zones are of the
/// *StopExcluded* type, so that detail has been omitted from the rule table:
///
///
/// <table>
///   <tr><th>Rule + Zone   <th>State `id`  <th>`type`        <th>`yield_to`
///   <tr><td>"EB/Straight" <td>"static"    <td>*Go*          <td>---
///   <tr><td>"WB/Straight" <td>"static"    <td>*Go*          <td>---
///   <tr><td>"NB/Right"    <td>"static"    <td>*StopThenGo*  <td>"EB/Straight"
///   <tr><td>"NB/Left"     <td>"static"    <td>*StopThenGo*  <td>"EB/Straight", "WB/Straight"
/// </table>
///
/// The `State::Id`'s chosen here ("static") are arbitrary.
///
/// As in the earlier Pedestrian Crosswalk example, the static *Go* rules
/// of the eastbound and westbound paths show that they always have the
/// right-of-way, but vehicles are still required to avoid stopping in the
/// intersection.  Traffic turning right onto the artery (following the
/// "NB/Right" path) must stop at the stop sign, and then yield to any
/// eastbound traffic.  Traffic turning left onto the artery must stop
/// and then yield to both eastbound and westbound traffic.
///
/// *Example: Protected/Permitted Left Turn*
///
/// @anchor RoWR-protected-left
/// @image html right-of-way-example-protected-left.svg "Intersection with protected/permitted left turn."
///
/// @ref RoWR-protected-left "Figure" provides a more complex scenario with a
/// dynamic signal-controlled intersection:
///   * The north-south street is one-way, northbound only.
///   * East-west traffic is two-way, with a dedicated left-turn lane for
///     eastbound traffic turning north.
///   * "Right Turn on Red" is allowed (which affects both northbound and
///     westbound vehicles).
///   * In the signaling cycle, eastbound traffic has both a protected-left
///     (green arrow) phase and a permitted-left (flashing yellow arrow) phase.
///   * There are a total of seven zones (paths) traversing the intersection
///     (illustrated by the seven arrows).
///
/// With seven zones, seven rule instances are required.  The rules have
/// been labeled by a combination of the initial heading and the turn
/// direction of their paths. (E.g., "NB/Left" refers to "the northbound
/// path that turns left".)  All the zones are of the /StopExcluded/ type,
/// so that detail has been omitted from the rule table:
///
/// <table>
///   <tr><th>Rule + Zone   <th>State `id`      <th>`type`       <th>`yield_to`
///   <tr><td rowspan="2"> "NB/Right"    <td>"Red"           <td>*StopThenGo* <td>"EB/Straight"
///   <tr>                               <td>"Green"         <td>*Go*         <td>---
///   <tr><th> <th> <th> <th>
///   <tr><td rowspan="2">"NB/Straight"  <td>"Red"           <td>*Stop*       <td>---
///   <tr>                               <td>"Green"         <td>*Go*         <td>---
///   <tr><th> <th> <th> <th>
///   <tr><td rowspan="2">"NB/Left"      <td>"Red"           <td>*Stop*       <td>---
///   <tr>                               <td>"Green"         <td>*Go*         <td>---
///   <tr><th> <th> <th> <th>
///   <tr><td rowspan="2">"EB/Straight"  <td>"Red"           <td>*Stop*       <td>---
///   <tr>                               <td>"Green"         <td>*Go*         <td>---
///   <tr><th> <th> <th> <th>
///   <tr><td rowspan="3">"EB/Left"     <td>"Red"           <td>*Stop*       <td>---
///   <tr>                              <td>"Green"         <td>*Go*         <td>---
///   <tr>                              <td>"FlashingYellow"<td>*Go*         <td>"WB/Straight", "WB/Right"
///   <tr><th> <th> <th> <th>
///   <tr><td rowspan="2">"WB/Right"    <td>"Red"           <td>*StopThenGo* <td>"NB/Straight", "EB/Left"
///   <tr>                              <td>"Green"         <td>*Go*         <td>---
///   <tr><th> <th> <th> <th>
///   <tr><td rowspan="2">"WB/Straight" <td>"Red"           <td>*Stop*       <td>---
///   <tr>                              <td>"Green"         <td>*Go*         <td>---
///   <tr><th> <th> <th> <th>
/// </table>
///
/// The `State::Id`'s have been chosen to loosely match the states of the
/// corresponding traffic signals.  (Note that typically a "yellow light"
/// confers the same right-of-way as a "green light"; the only difference
/// is that the yellow indicates that a transition to red is imminent.)
///
/// Each rule has at least two states.  The straight-ahead rules
/// ( `*`/Straight ) and the northbound left-turning rule ( NB/Left ) are quite
/// straightforward: either "Stop" with no right-of-way or "Go" with full
/// right-of-way.  The other turning rules are a bit more interesting.
///
/// Since "Right Turn on Red" is allowed, both the "NB/Right" and "WB/Right"
/// rules have *StopThenGo* states (instead of *Stop* states) that must
/// yield to other traffic.  "NB/Right" must yield to eastbound traffic,
/// and "WB/Right" must yield to northbound traffic.
///
/// The "EB/Left" rule has two *Go* states.  One is the protected turn state, in
/// which the left turn is given full priority over oncoming westbound traffic.
/// The other is the permitted turn state, in which the left turn must yield
/// to westbound traffic.  In the US, a possible traffic light configuration
/// for such an intersection would signal the protected turn by a solid
/// green arrow, and the permitted turn by a flashing yellow arrow.
///
/// *Example: Freeway Merge*
///
/// @anchor RoWR-freeway-merge
/// @image html right-of-way-example-freeway-merge.svg "Entrance ramp merging onto a 2-lane (one-way) freeway."
///
/// @ref RoWR-freeway-merge "Figure" is a scenario with a freeway merge:
///   * Freeway has two lanes of eastbound traffic.
///   * Entrance ramp merges onto the freeway from the right (south).
///   * Merging traffic must yield to traffic already on the freeway.
///   * Two zones traverse the area where the merge occurs (illustrated by
///     the two arrows).
///
/// This is a static scenario with two static rules:
///
/// <table>
///   <tr><th>Rule + Zone <th>`zone_type`   <th>State `id` <th>`type` <th>`yield_to`
///   <tr><td>"Freeway"   <td>`StopAllowed` <td>"static"   <td>*Go*   <td>---
///   <tr><td>"Entrance"  <td>`StopAllowed` <td>"static"   <td>*Go*   <td>"Freeway"
/// </table>
///
/// The `State::Id`'s chosen here ("static") are arbitrary.
///
/// The only constraint encoded by these two rules is that the "Entrance"
/// traffic should yield to the "Freeway" traffic.  Note that unlike
/// previous examples, both zones in this scenario have a zone-type of
/// `StopAllowed`.  That means there are no "stop lines" (real or
/// implicit) and no exclusion zones that are expected to be left
/// unblocked by stopped traffic.  Both rules' static states are of type
/// *Go*, as well; neither path is expected to stop.  Ideally, the entrance
/// traffic never stops, but instead speeds up to seamlessly merge into
/// the freeway flow.
///
/// > TODO `DirectionUsageRule`:
/// `DirectionUsageRule`: Direction Usage
///
/// *Captures allowed direction-of-travel.*
///
///    * id
///    * zone (`LaneSRange`)
///    * allowed use:
///      * *bidirectional* (e.g., non-striped single-lane residential street)
///      * *unidirectional, s increasing*
///      * *unidirectional, s decreasing*
///      * *bidirectional, turning-only*
///      * *no-traffic* (e.g., median strip)
///      * *parking-lane*
///    * time-of-day/calendar condition?
///
/// > TODO `LaneChangeRule`: Lane-change/Passing Restrictions
/// `LaneChangeRule`: Lane-change/Passing Restrictions
///
/// *Captures restrictions on lateral/adjacent lane transitions.*
///    * id
///    * zone (`LaneSRange`)
///    * applicable direction
///      * to-left
///      * to-right
///    * constraint
///      * allowed
///      * forbidden
///      * *discouraged?* (e.g., to capture solid white lines separating turn
///        lanes from through traffic)
///    * *Should this capture "passing vs lane-change" purpose, too, (e.g.,
///      the white-vs-yellow distinction) or should that just be implied by
///      `DirectionUsageRule`?*
///    * time-of-day/calendar condition?
///
/// > TODO `OngoingRouteRule`: "Turning" Restrictions
/// `OngoingRouteRule`: "Turning" Restrictions
///
/// *Captures restrictions on longitudinal/end-to-end lane transitions.*
///    * id
///    * applicable originating `LaneIdEnd`
///    * ongoing `LaneIdEnd`
///    * restricted vehicle type
///      * (not) any
///      * (not) bus
///      * (not) truck
///      * ...
///    * time-of-day/calendar condition?
///    * *(Or, maybe this concept is better represented by vehicle restrictions
///      on the ongoing lane instead.)*
///
/// > TODO `PreferentialUseRule`: Vehicle Restrictions
/// `PreferentialUseRule`: Vehicle Restrictions
///
/// *Captures vehicle-type traffic restrictions.*
///    * id
///    * zone (`LaneSRange`)
///    * vehicle type
///      * high-occupancy vehicles (HOV) only
///      * no trucks
///      * bus only
///      * emergency vehicles only
///      * etc
///    * time-of-day/calendar condition?
///    * *Should this should be merged with `DirectionUsageRule`, because
///      lane usage/direction might be specified per vehicle type?*
///
/// @subsection phase_dynamics `Phase` dynamics: how to handle the rule state changes
///
/// As explained before, rule state dynamics are complex enough to be modeled
/// within the very same `Rule` type. Also, the `RoadRulebook` falls short to
/// provide query support for the state transition of the Rules it can fetch.
/// `maliput` models the sequencing of rule states and traffic lights' bulbs
/// as a ring of `Phases`. Each `Phase` holds a dictionary of rule IDs to
/// rule states (`DiscreteValues`) and related bulb IDs (`UniqueBulbIds`) to the
/// bulb state (`BulbState`).
///
/// `Phases` have an implicit rule co-location constraint. Sequencing of
/// `Phases` that do not share the same region are designer's responsibility.
/// The API does not provide any further enhanced semantics to deal with such
/// case. However, the ordered sequence of co-located `Phases` is captured by a
/// `PhaseRing`. `Phases` are stored in `PhaseRings` which cannot be empty. They
/// may have only one `Phase` though, meaning that it is self connected or it
/// never ends.
///
/// > NOTE: Intra-`Phase` relationship between `Rule::Id` and `UniqueBulbId` can
/// > be discovered by the actual `DiscreteValue`'s related unique IDs by the
/// > agent.
///
/// The `PhaseRing` acts as a container of all the related `Phases` in a
/// sequence. A designer might query them by the `Phase::Id` or the next
/// `Phases`, but no strict order should be expected. Instead, `PhaseProvider`
/// offers an interface to obtain the current and next `Phase::Ids` for a
/// `PhaseRing`. Custom time based or event driven behaviors could be
/// implemented for this interface. Similarly to the rules, there are convenient
/// "manual" implementations to exercise the interfaces in integration examples.
///
/// @subsection traffic_lights `TafficLight` modelling and databases
///
/// Traffic lights physically come in various shapes and configurations. With a
/// set of bulbs (from now on `BulbGroups` and `Bulbs`) that are semantically
/// related and serve many road lanes simultaneously. We are not able to capture
/// all variations of configurations, but the API proposes a generic enough
/// abstraction that comprises most of the use cases that involve the physical
/// properties of the `TrafficLights`, their `BulbGroups` and each `Bulb` in
/// them.
///
/// `Bulbs` can be round or arrows, have an orientation and position relative to
/// the parent `BulbGroup` and a specific color. At certain moment, they can be
/// on, off or blinking. Each `Bulb` occupies a volume defined with a bounding
/// box.
///
/// A collection of `Bulbs` compose a `BulbGroup`. Each `BulbGroup` has a pose
/// relative to the `TrafficLight` they are part of.
///
/// Finally, a `TrafficLight` is composed of a set of `BulbGroups`. It is set in
/// a pose (position and orientation) in the Inertial Frame. Once it is built,
/// both its `BulbGroups` and `Bulbs` are uniquely identified by composing the
/// parent (`TrafficLight::Id`), child (`BulbGroup::Id`) and grandchild
/// (`Bulb::Id`) IDs. Those compound IDs are represented by `UniqueBulbId` and
/// `UniqueBulbGroupId` types.
///
/// `TrafficLights` can be obtained via `TrafficLightBook`, another interface to
/// which offers queries by `TrafficLight::Id` and to retrieve all
/// `TrafficLights`.
///
/// @subsection intersections_aggregation `Intersections` to aggregate multiple related entities
///
/// The inherent complexity of the rule dynamics require multiple interfaces and
/// and concrete implementations to be setup in favor of describing the traffic
/// dynamics and adapt to multiple scenarios. In favor of reducing the
/// complexity of the queries, the `Intersection` aggregates multiple disperse
/// IDs and related entities that would require many queries to different books
/// and state providers.
///
/// To obtain the `Intersection` the agent is immerse, it can use the
/// `IntersectionBook` which provides multiple query semantics to retrieve the
/// right one:
/// - by `Intersection::Id`
/// - by `TrafficLight::Id`
/// - by `DiscreteValueRule::Id`
/// - all `Intersections`
/// - (DEPRECATED) by `RightOfWayRule::Id`
///
/// > TODO Furniture and Physical Features
/// Furniture and Physical Features
///
/// *Provide a database of physical features with spatial location and extent.*
///
/// In many cases these are related to rules in the `RoadRulebook` (e.g., signs
/// and stripes are indicators for rules of the road).
///    * linear features
///      * striping
///    * areal features
///      * crosswalks
///      * restricted medians
///      * do-not-block zones
///    * signage
///      * stop lights, stop signs
///      * turn restrictions
///    * other (volumetric) furniture
///      * benches
///      * mailboxes
///      * traffic cones
///      * refrigerator that fell off a truck
///    * potholes
