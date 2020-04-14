// Copyright 2020 Toyota Research Institute
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
///    system referred to as the *world frame*.
///  * The road surface is a bounded compact orientable 2-dimensional manifold
///    embedded in the @f$ \mathbb{R}^3 @f$ world frame via a @f$ G^1 @f$
///    continuous map from @f$ \mathbb{R}^2 \to \mathbb{R}^3 @f$.
///  * The road surface is extended via its normals into a bounded compact
///    orientable 3-dimensional road volume, also embedded in the
///    @f$ \mathbb{R}^3 @f$ world frame via a @f$ G^1 @f$ continuous map from
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
/// volume from a local *lane frame* into the world frame.  `Lanes` are
/// connected at `BranchPoints`, and the graph of `Lanes` and
/// `BranchPoints` describes the topology of a `RoadGeometry`. `Segments` which map
/// to intersecting volumes of the world frame (e.g., intersections) are grouped
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
/// @subsubsection world_frame_versus_lane_frame `World` Frame versus `Lane` Frame
///
/// Two types of coordinate frames are used in this model: the (single)
/// `World`-frame and the (multiple) `Lane`-frames.  In both, distances
/// are typically measured in units of meters.
///
/// The `World`-frame is any right-handed 3D inertial Cartesian coordinate
/// system, with orthonormal basis @f$(\hat{x},\hat{y},\hat{z})@f$ and
/// positions expressed as triples @f$(x,y,z)@f$.  This could be a
/// globally-flat coordinate system, e.g., ECEF ("Earth-centered,
/// Earth-fixed").  Or, it could be a locally-flat projection of the
/// Earth's surface, e.g., a UTM ("Universal Transverse Mercator")
/// projection coupled with elevation.  No specific projection is mandated
/// by `maliput`.
///
/// > *Currently:*  @f$\hat{z}@f$ is assumed to be *up*, with @f$z@f$ representing an
/// > altitude or elevation.  @f$\hat{x}@f$ and @f$\hat{y}@f$ span the horizontal
/// > plane.  Typically, the "ENU" convention is used: @f$\hat{x}@f$ points *East*
/// > and @f$\hat{y}@f$ points *North*.
/// >
/// > *In the future:* the `maliput` API will be extended to provide a
/// > description of the geographic coordinate system (if any) used by a
/// > `RoadGeometry`, as well as a local gravity vector as a function of
/// > position.
///
/// A `Lane`-frame is a right-handed orthonormal curvilinear coordinate system, with
/// positions expressed as coordinates @f$(s,r,h)@f$.  Each `Lane` in a `RoadGeometry`
/// defines its own embedding into the `World`, and thus each `Lane`
/// has its own `Lane`-frame.
///
/// When embedded into the `World`, @f$s@f$ represents longitudinal distance
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
/// road manifold into the `World`, as a union of the per `Lane` maps.
/// This @f$W@f$ is technically an *immersion* and not an *embedding* because
/// it is not necessarily 1-to-1; as described later on, multiple `Lanes`
/// in the same `Segment` will double-cover the same region of the
/// @f$\mathbb{R}^3@f$ world frame.  Also, due to our representation of
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
/// @subsubsection lanes_as_lanes Lanes as `Lanes`
///
/// A `Lane` represents a lane of travel in a road network, expressing a path
/// along a stretch of asphalt as well as a parameterization of that asphalt
/// from one lateral edge to the other (including adjacent lanes of travel,
/// shoulders, etc).
///
/// As discussed above, a `Lane`, identified by @f$L@f$, defines a map @f$W_L@f$
/// from curvilinear coordinates to the `World`-frame:
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
/// `Lane`-frame and the `World`-frame).  The @f$s=0@f$ end of a `Lane` is
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
/// the @f/segment bounds@f/ for the `Lane`, the valid domain of @f$r@f$, which
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
/// @subsubsection lanes_joined_via_branchpoints Lanes Joined End-to-End via `BranchPoints`
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
///  1. The location of a `BranchPoint` is a well-defined point in the World frame.
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
/// @subsubsection adjacent_lanes_grouped_into_segments Adjacent Lanes Grouped into `Segments`
///
/// In real roads, the pavement is often divided into multiple adjacent
/// lanes of travel; in `maliput`, adjacent `Lanes` are grouped together
/// into `Segments`.  The basic idea is that a `Segment` corresponds to a
/// longitudinal stretch of pavement, and each `Lane` in that `Segment`
/// presents a different @f$(s,r,h)@f$ parameterization of that same pavement.
///
/// We would like for the segment-bounds of each `Lane` to map to the
/// same extent of physical space in the World frame, but that isn't always
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
/// @subsubsection intersecting_segments_grouped_into_junctions Intersecting `Segments` Grouped into `Junction`
///
/// It is possible for multiple `Segments` to cover the same pavement.
/// In fact, that is how intersections are represented, by criss-crossing
/// `Segments` which define the different paths through an intersection.
/// Overlapping `Segments` also occur where the road merges or diverges,
/// such as on-ramps, exit ramps, traffic circles, and a road that splits
/// to go around an impassable monument.
///
/// `Segments` which map to intersecting volumes in the World frame (in
/// terms of the union of the segment-bounds of their `Lanes`) are
/// grouped together into a `Junction`.  The primary (sole?) purpose of a
/// `Junction` is to indicate that objects in its component `Segments` may
/// spatially interact with each other (e.g., collide!).  Conversely, if
/// two `Segments` belong to two distinct `Junction`, then objects within
/// their respective segment-bounds should /not/ be touching.  (Note
/// that in considering intersection, we ignore the overlaps
/// that may occur where `Segments` join end-to-end via their `Lanes`.)
///
/// Every `Segment` must belong to one and only one `Junction`, and a
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
/// @subsection abstract_geometry_api_roadgeometry Abstract Geometry API: `RoadGeometry`
///
/// > TODO:  Explain semantics of object ID's.  (cross-referencing, tiling,
/// > debugging, visualization)
///
/// > TODO:  Reference to `maliput::api` doxygen.
///
/// @subsubsection basic_types Basic Types
///
/// * `GeoPosition`
/// * `LanePosition`
/// * `RoadPosition`
/// * ...
///
/// @subsubsection roadgeometry `RoadGeometry`
///
/// * accessors for component `Junctions`
/// * accessors for component `BranchPoints`
/// * accessors for characteristic lengths and tolerances
///   * `linear_tolerance`
///   * `angular_tolerance`
///   * `scale_length`
///
/// @subsubsection junction `Junction`
///
/// * accessors for parent `RoadGeometry`, component `Junctions`
///
/// @subsubsection segment `Segment`
///
/// * accessors for parent `Junction`, component `Lanes`
///
/// @subsubsection lane `Lane`
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
/// @subsubsection branchpoints `BranchPoints`
///
/// * accessors for `Lanes` on each side ("A" versus "B")
/// * accessor for the set of confluent `Lanes` for a given `Lane`
/// * accessor for the set of ongoing `Lanes` for a given `Lane`
/// * accessor for the default branch (ongoing `Lane`) for a given `Lane`
/// * accessor for parent `RoadGeometry`
///
/// @subsection concrete_implementation_maliput_multilane Concrete Implementation: `maliput::multilane`
///
/// So-named because it admits multiple `Lanes` per
/// `Segment`, an advance over its predecessor (`monolane`) which only
/// admitted a single `Lane` per `Segment`.
///
/// `multilane`  is an implementation of the
/// `maliput` geometry API which synthesizes a road network from a small set
/// of primitive building blocks, mimicking techniques used in the geometric
/// design of real roads.  The basic geometry of a `Segment` is derived
/// from the combination of a plane curve, an elevation
/// function, and a superelevation function, combined together to define a
/// ruled surface.  A `Segment` has a longitudinal *reference curve*
/// (similar to a `Lane`'s centerline) and each of the `Lanes` of a
/// `Segment` is defined via a constant lateral offset, along the segment
/// surface, from that reference curve.
///
/// Three coordinate frames are involved in the following discussion:
///  * @f$(x,y,z)@f$ is a position in the `World`-frame.
///  * @f$(s,r,h)_{LANE,i}@f$ is a position in the `Lane`-frame (discussed
///    in section @ref world_frame_versus_lane_frame ) of the `Lane` with
///    index @f$i@f$.
///  * @f$(p,r,h)_{SEG}@f$ is a position in a curvilinear reference frame of
///    the `Segment`, analogous to @f$(s,r,h)_{LANE,i}@f$ for a `Lane`.
///    The parameter @f$p_{SEG} \in [0, 1]@f$ spans the `Segment` longitudinally.
///    @f$r_{SEG}@f$ is a lateral offset from the `Segment`'s reference curve,
///    along the `Segment` surface. @f$h_SEG@f$ is height above the surface.
///
/// @subsubsection segment_geometry `Segment` Geometry
///
/// > TODO Reconsider the use of the word "geometry" below.
/// > The geometry of a `Segment` is completely derived from a map
/// >
/// > @f[
/// > W: (p,r,h)_{SEG} \mapsto (x,y,z)
/// > @f]
/// >
/// > which we will construct in stages, starting with the `Segment` reference curve
/// >
/// > @f[
/// > W(p_{SEG}) \equiv W(p_{SEG},0,0),
/// > @f]
/// >
/// > followed by the `Segment` surface
/// >
/// > @f[
/// > W(p_{SEG},r_{SEG}) \equiv W(p_{SEG},r_{SEG},0).
/// > @f]
///
/// The construction of @f$W(p_{SEG},r_{SEG},h_{SEG})@f$ will involve
/// three fundamental functions, @f$G_\text{xy}@f$, @f$G_z@f$, and @f$\Theta@f$.
///
/// The first fundamental function @f$G_\text{xy}@f$ defines a two dimensional
/// *planar primitive curve* in the @f$xy@f$ -plane:
///
/// @f[
/// G_{xy}: p_{SEG} \mapsto (x,y).
/// @f]
///
/// This curve establishes the basic geometric primitive of the `Segment`
/// (e.g., "constant-radius arc").
/// We define @f$l@f$ as a path-length along this plane curve, in the range
/// @f$[0, l_\text{max}]@f$, where @f$l_\text{max}@f$ is the total path-length
/// of the curve.  @f$G_{xy}@f$ is specifically parameterized such that
///
/// @f[
/// p_{SEG} \equiv \frac{l}{l_\text{max}};
/// @f]
///
/// in other words, @f$p_{SEG}@f$ is linear in path-length along the planar
/// primitive curve and @f$p_{SEG} \in [0,1]@f$.
///
/// The second fundamental function @f$G_z@f$ specifies elevation above the
/// @f$(xy)@f$-plane (albeit with a peculiar scale factor):
///
/// @f[
/// G_z: p_{SEG} \mapsto \frac{1}{l_\text{max}}z
/// @f]
///
/// Taking @f$G_{xy} = (G_x, G_y)@f$ and @f$G_z@f$ together,
///
/// > @f[
/// > \left(\begin{array}{c} G_{xy}\\ l_\text{max}G_z \end{array}\right): p_{SEG} \mapsto
/// >   \left(\begin{array}{c}x\\y\\z\end{array}\right)
/// > @f]
///
/// @f[
/// \left(\begin{array}{c}x\\y\\z\end{array}\right) =
/// W(p_{SEG}) =
/// \left(\begin{array}{c} G_x(p_{SEG})\\
///                        G_y(p_{SEG})\\
///                        l_\text{max}G_z(p_{SEG}) \end{array}\right)
/// @f]
///
/// defines the three dimensional *reference curve* @f$W(p_{SEG})@f$ for the `Segment`.
/// @f$G_z@f$ is constructed with the scale factor of @f$1/l_\text{max}@f$ specifically
/// so that:
///
/// @f[
/// \begin{eqnarray*}
///       z & = & l_\text{max} G_z(p_{SEG})\\
///         & = & l_\text{max} G_z\left(\frac{l}{l_\text{max}}\right)\\
/// \dot{z} & = & \frac{dz}{dl} = \frac{d}{dp_{SEG}}G_z(p_{SEG})
/// \end{eqnarray*}
/// @f]
///
/// This allows us to derive the first derivative of @f$G_z@f$ directly from
/// the `World`-frame slope @f$\dot{z} = \frac{dz}{dl}@f$ of the segment
/// surface along its reference curve.  This is convenient because @f$\dot{z}@f$
/// is what a road designer would nominally specify as the "slope of the road"
/// or the "grade of the road".
///
/// The third fundamental function @f$\Theta@f$ specifies the superelevation of
/// the `Segment` surface:
///
/// @f[
/// \Theta: p_{SEG} \mapsto \frac{1}{l_\text{max}}\theta
/// @f]
///
/// Superelevation @f$\theta@f$ is the "twist" in a road, given as a right-handed
/// angle of rotation around the tangent of the reference curve @f$W(p_{SEG})@f$.
/// Zero superelevation leaves the surface parallel with the
/// @f$xy@f$ plane. Note that superelevation becomes ambiguous when the
/// tangent of the reference curve points in the @f$\hat{z}@f$ direction.
///
/// As with @f$G_z@f$, @f$\Theta@f$ is scaled so that:
///
/// @f[
/// \begin{eqnarray*}
///       \theta & = & l_\text{max} \Theta\left(\frac{l}{l_\text{max}}\right)\\
/// \dot{\theta} & = &
///               \frac{d\theta}{dl} = \frac{d}{dp_{SEG}}\Theta(p_{SEG})
/// \end{eqnarray*}
/// @f]
///
/// > With the three fundamental functions in hand, we can express the orientation
/// > of the @f$(\hat{p},\hat{r},\hat{h})_{SEG}@f$ frame along the reference curve,
/// > with respect to the `World`-frame, as a roll/pitch/yaw rotation:
///
/// We use all three fundamental functions to define a rotation
///
/// @f[
/// \begin{align*}
/// \mathbf{R}(p_{SEG}) &=
///  \mathbf{R}_{\gamma(p_{SEG})}
///  \mathbf{R}_{\beta(p_{SEG})} \mathbf{R}_{\alpha(p_{SEG})}
/// \end{align*}
/// @f]
///
/// where
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\gamma(p_{SEG})} &=
///   \left(\begin{array}{rrr}
///   \cos\gamma & -\sin\gamma & 0 \\
///   \sin\gamma &  \cos\gamma & 0 \\
///            0 &           0 & 1
///   \end{array}\right) & \text{(yaw)}\\
/// \end{align*}
/// @f]
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\beta(p_{SEG})}  &=
///   \left(\begin{array}{rrr}
///    \cos\beta & 0 & \sin\beta \\
///            0 & 1 &         0 \\
///   -\sin\beta & 0 & \cos\beta
///   \end{array}\right) & \text{(pitch)} \\
/// \end{align*}
/// @f]
///
/// @f[
/// \begin{align*}
///   \mathbf{R}_{\alpha(p_{SEG})} &=
///   \left(\begin{array}{rrr}
///   1 &          0 &           0 \\
///   0 & \cos\alpha & -\sin\alpha \\
///   0 & \sin\alpha &  \cos\alpha
///   \end{array}\right) & \text{(roll)}
/// \end{align*}
/// @f]
///
/// and
///
/// @f[
/// \begin{align*}
/// \gamma(p_{SEG}) &=
///   \mathrm{atan2}\negthickspace\left(\frac{dG_y}{dp_{SEG}},
///                       \frac{dG_x}{dp_{SEG}}\right) & \text{(yaw)}\\
/// \beta(p_{SEG})  &=
///   \arctan\negthickspace\left(\frac{dG_z}
///                                         {dp_{SEG}}\right)
/// & \text{(pitch)} \\
/// \alpha(p_{SEG}) &= l_\text{max}\Theta(p_{SEG}) & \text{(roll)}
/// \end{align*}
/// @f]
///
/// > Note that @f$\hat{p}_{SEG}@f$ is solely determined by @f$W(p_{SEG})@f$,
/// > and as expected,
/// > @f$\hat{p}_{SEG} = \frac{W'(p_{SEG})}{\lVert W'(p_{SEG})\rVert}@f$.
///
/// With @f$\mathbf{R}(p_{SEG})@f$ , we can extend the `Segment` reference curve @f$W(p_{SEG})@f$
/// to construct the `Segment` *surface* @f$W(p_{SEG}, r_{SEG})@f$ as:
///
/// @f[
/// \begin{align*}
/// \left(\begin{array}{c}x\\y\\z\end{array}\right) =
/// W(p_{SEG},r_{SEG}) = \left(
/// \begin{array}{c}
///    G_{xy}(p_{SEG})\\
///    l_\text{max} G_z(p_{SEG})
/// \end{array} \right) +
/// \mathbf{R}(p_{SEG})\negthickspace
/// \begin{pmatrix}
/// 0\\ r_{SEG} \\ 0 \end{pmatrix}.
/// \end{align*}
/// @f]
///
/// This function defines a *ruled surface*.  For any @f$p_{SEG}@f$,
/// @f$W(p_{SEG},r_{SEG})@f$ is linear in @f$r_{SEG}@f$ and motion along
/// @f$r_{SEG}@f$ is in a straight line.
///
/// Now that we have the surface embedding @f$W(p_{SEG},r_{SEG})@f$,
/// we can derive
/// the basis vectors @f$(\hat{p}, \hat{r}, \hat{h})_{SEG}@f$ along the surface
/// and the corresponding orientation @f$\mathbf{R}(p_{SEG},r_{SEG})@f$:
///
/// @f[
/// \begin{align*}
/// \mathbf{R}(p_{SEG},r_{SEG}) &=
///                      \begin{pmatrix}\hat{p} & \hat{r} & \hat{h}\end{pmatrix}\\
/// \hat{p}_{SEG} &=
///  \frac{\partial_{p_{SEG}} W(p_{SEG},r_{SEG})}{\lVert\partial_{p_{SEG}} W(p_{SEG},r_{SEG})\rVert}\\
/// \hat{r}_{SEG} &=
///  \frac{\partial_{r_{SEG}} W(p_{SEG},r_{SEG})}{\lVert\partial_{r_{SEG}} W(p_{SEG},r_{SEG})\rVert}\\
/// \hat{h}_{SEG} &= \hat{p}_{SEG} \times \hat{r}_{SEG}
/// \end{align*}
/// @f]
///
/// A few things are worth noting at this point:
///
///  * @f$\hat{r}_{SEG} = \mathbf{R}(p_{SEG}) \begin{pmatrix}0\\1\\0\end{pmatrix}@f$.
///    Thus, @f$\hat{r}_{SEG}@f$ is independent of @f$r_{SEG}@f$.
///  * @f$\mathbf{R}(p_{SEG},r_{SEG}) = \mathbf{R}(p_{SEG})@f$ along
///    @f$r_{SEG} = 0@f$ just as it should be; the orientation along the
///    `Segment`'s reference curve is consistent in both expressions.
///  * @f$\hat{p}_{SEG}@f$ is *not necessarily* independent of
///    @f$r_{SEG}@f$.  Consequently, @f$\mathbf{R}(p_{SEG},r_{SEG})@f$ is not
///    necessarily equal to @f$\mathbf{R}(p_{SEG})@f$ for
///    @f$r_{SEG}\ne 0@f$.  This will become important when we try to
///    join `Segments` end-to-end preserving @f$G^1@f$ continuity, discussed in
///    section @ref ensuring_g1_contiguity .
///
/// *Finally*, with @f$\mathbf{R}(p_{SEG},r_{SEG})@f$ in hand (and points 1 and
/// 2 above), we can define the complete volumetric world map
/// @f$W(p_{SEG},r_{SEG},h_{SEG})@f$ for a `Segment`'s geometry:
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix}x\\y\\z\end{pmatrix} = W(p_{SEG},r_{SEG},h_{SEG}) = \left(
/// \begin{array}{c}
///    G_x(p_{SEG})\\
///    G_y(p_{SEG})\\
///    l_\text{max} G_z(p_{SEG})
/// \end{array} \right) +
/// \mathbf{R}(p_{SEG},r_{SEG})\negthickspace
/// \begin{pmatrix}
/// 0\\ r_{SEG} \\ h_{SEG} \end{pmatrix}.
/// \end{align*}
/// @f]
///
/// This is simply @f$W(p_{SEG},r_{SEG})@f$ displaced by @f$h_{SEG}@f$ along
/// the surface normal @f$\hat{h}_{SEG}@f$.
///
/// @subsubsection lane_geometry `Lane` Geometry
///
/// A `Lane` derives its geometry from its `Segment`.  In `multilane`, the
/// centerline of the `Lane` with index @f$i@f$ is a parallel curve with a constant
/// lateral offset @f$r_i@f$ from the reference curve (at @f$r_{SEG} = 0@f$) of the
/// `Segment`.  We can express this relationship as a transform between
/// @f$(s,r,h)_{LANE,i}@f$ (`Lane`-frame) and @f$(p,r,h)_{SEG}@f$
/// (`Segment`-frame):
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix} p_{SEG}\\
///                 r_{SEG}\\
///                 h_{SEG} \end{pmatrix}
/// &= \begin{pmatrix}    P(s_{LANE,i})\\
///                    r_{LANE,i} + r_i\\
///                          h_{LANE,i} \end{pmatrix}
/// \end{align*}
/// @f]
///
/// The tricky part here is @f$P:s_{LANE,i} \mapsto p_{SEG}@f$, which relates
/// @f$s_{LANE,i}@f$ to @f$p_{SEG}@f$, and involves the
/// path-length integral over @f$W(p_{SEG},r_{SEG})@f$.
///
/// `maliput` defines @f$s_{LANE,i}@f$ as the path-length along a `Lane`'s
/// centerline, and in `multilane` that centerline is a curve with constant
/// @f$r_{SEG} = r_i@f$.  Thus:
///
/// @f[
/// \begin{align*}
/// s_{LANE,i} = S(p_{SEG}) &=
///  \left. \int \left\lVert \partial_{p_{SEG}}W(p_{SEG}, r_{SEG})
///  \right\rVert dp_{SEG} \right\rvert_{r_{SEG} = r_i}.
/// \end{align*}
/// @f]
///
/// The function @f$P@f$ that we need is the inverse of the path-integral @f$S@f$.
///
/// Unfortunately, there is generally no closed-form solution for either
/// @f$S@f$ or @f$P@f$, particularly if the surface is not flat.  `multilane` will
/// compute @f$P(s_{LANE,i})@f$ and @f$S(p_{SEG})@f$ analytically if
/// possible (e.g., for some flat surfaces) and otherwise will use more costly
/// numerical methods to ensure accurate results. Which makes us
/// wonder, perhaps the `Lane`-frame of `maliput` would be better off
/// using an arbitrary longitudinal parameter @f$p_{LANE,i}@f$ which could
/// be converted to a distance @f$s_{LANE,i}@f$ on demand, instead of the other
/// way around.
///
/// > TODO: Derivation of orientation at arbitrary @f$(s,r,h)_{LANE,i}@f$ point.
/// >
/// > TODO: Derivation of motion-derivatives.
/// >
/// > TODO: Derivation of surface/path curvatures.
///
/// ### Available Implementations of @f$G_\text{xy}@f$, @f$G_z@f$, and @f$\Theta@f$
///
/// `multilane` currently implements one form for each of @f$G_{xy}@f$,
/// @f$G_z@f$, and @f$\Theta@f$.  @f$G_{xy}@f$ is implemented for a constant curvature
/// arc (which includes zero curvature, i.e., straight line segments).
/// Elevation @f$G_z@f$ and superelevation @f$\Theta@f$ are implemented for cubic
/// polynomials.  These forms were chosen because they provide the smallest,
/// simplest set of primitives that allow for the assembly of fully
/// three-dimensional road networks that maintain @f$G^1@f$ continuity across
/// segment boundaries.
///
/// The exact form that @f$G_{xy}@f$ takes is:
///
/// @f[
/// \begin{align*}
/// \begin{pmatrix} x\\ y \end{pmatrix} = G_\text{xy}(p_{SEG}) &=
///     \begin{pmatrix}x_0\\ y_0\end{pmatrix} +
///     \left\lbrace \begin{array}
///         \frac{1}{\kappa}\begin{pmatrix}
///           \cos(\kappa l_\text{max} p_{SEG} + \gamma_0 - \frac{\pi}{2}) - \cos(\gamma_0 - \frac{\pi}{2})\\
///           \sin(\kappa l_\text{max} p_{SEG} + \gamma_0 - \frac{\pi}{2}) - \sin(\gamma_0 - \frac{\pi}{2})
///           \end{pmatrix} & \text{for }\kappa > 0\\
///         l_\text{max} p_{SEG}
///           \begin{pmatrix}\cos{\gamma_0}\\ \sin{\gamma_0}\end{pmatrix}
///           & \text{for }\kappa = 0\\
///         \frac{1}{\kappa}\begin{pmatrix}
///           \cos(\kappa l_\text{max} p_{SEG} + \gamma_0 + \frac{\pi}{2}) - \cos(\gamma_0 + \frac{\pi}{2})\\
///           \sin(\kappa l_\text{max} p_{SEG} + \gamma_0 + \frac{\pi}{2}) - \sin(\gamma_0 + \frac{\pi}{2})
///           \end{pmatrix} & \text{for }\kappa < 0\\
///     \end{array} \right\rbrace
/// \end{align*}
/// @f]
///
///
/// where @f$\kappa@f$ is the signed curvature (positive is
/// counterclockwise/leftward), @f$l_\text{max}@f$ is the arc length,
/// @f$\begin{pmatrix}x_0\\y_0\end{pmatrix}@f$ is the
/// starting point of the arc, and @f$\gamma_0@f$ is the initial yaw of the
/// (tangent) of the arc (with @f$\gamma_0 = 0@f$ in the @f$+\hat{x}@f$
/// direction).  Note that the @f$\kappa = 0@f$ expression is simply a line
/// segment of length @f$l_\text{max}@f$, and it is the limit of the @f$\kappa
/// \neq 0@f$ expressions as @f$\kappa \to 0@f$.
///
/// With regards to geometric road design, a constant curvature
/// @f$G_\text{xy}@f$ does not provide a complete toolkit.  Most road designs
/// involve clothoid spirals, which are plane curves with curvature that
/// is /linear/ in path length.This is so that vehicles can navigate
/// roads using continuous changes in steering angle, and, likewise, so that
/// their occupants will experience continuous changes in radial acceleration.
/// `multilane` is expected to extend support for clothoid @f$G_\text{xy}@f$
/// in the future.
///
/// For @f$G_z@f$ and @f$\Theta@f$, a cubic polynomial is the lowest-degree polynomial
/// which allows for independently specifying the value and the first derivative
/// at both endpoints.  Thus, @f$G_z@f$ takes the form:
///
/// @f[
/// \begin{align*}
/// \begin{split}
/// \frac{1}{l_\text{max}}z = G_z(p_{SEG}) &=
///  \frac{z_0}{l_\text{max}} +
///  \dot{z_0} p_{SEG} +
///  \left(\frac{3(z_1 - z_0)}{l_\text{max}} - 2\dot{z_0} - \dot{z_1}\right)
///    p_{SEG}^2 \\
///  &\quad + \left(\dot{z_0} + \dot{z_1} - \frac{2(z_1 - z_0)}{l_\text{max}}\right)
///    p_{SEG}^3
/// \end{split}
/// \end{align*}
/// @f]
///
/// where @f$z_0@f$ and @f$z_1@f$ are the initial and final elevation
/// respectively, and @f$\dot{z_0}@f$ and @f$\dot{z_1}@f$ are the initial and
/// final @f$\frac{dz}{dl}@f$, which is simply the slope of the road as
/// measured by the intuitive "rise over run".  @f$\Theta@f$ has an identical
/// expression, with every @f$z@f$ replaced by @f$\theta@f$.  Note that
/// @f$\dot{\theta} = \frac{d\theta}{dl}@f$, the rate of twisting of the road,
/// is not particularly intuitive, but that's ok because in general
/// @f$\dot{\theta_0}@f$ and @f$\dot{\theta_1}@f$ will be set by `multilane` and
/// not by the road designer, as we will see in section @ref ensuring_g1_contiguity .
///
/// @subsubsection ensuring_g1_contiguity Ensuring G¹ Continuity
///
/// > TODO:  Tell me more!
///
/// @subsubsection builder_helper_interface `Builder` helper interface
///
/// Users are not expected to assemble a `multilane::RoadGeometry` by
/// constructing individual instances of `multilane::Lane`, etc, by hand.
/// Instead, `multilane` provides a `Builder` interface which handles
/// many of the constraints involved in constructing a valid `RoadGeometry`.
///
/// > TODO:  Tell me more!
///
/// @subsubsection yaml_file_format YAML file format
///
/// > TODO:  Tell me more!
///
