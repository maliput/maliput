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
/// described later in Section **RoadRulebook**.
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
