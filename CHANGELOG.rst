^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package maliput
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.1 (2025-09-03)
------------------
* Range validator error type template. (`#673 <https://github.com/maliput/maliput/issues/673>`_)
* Contributors: Santiago Lopez

1.7.0 (2025-08-27)
------------------
* Bazel: Resolve linking in downstream packages. (`#670 <https://github.com/maliput/maliput/issues/670>`_)
* Rules related errors. (`#667 <https://github.com/maliput/maliput/issues/667>`_)
* Downgrade logging message. (`#671 <https://github.com/maliput/maliput/issues/671>`_)
* Make a template for Throw method. (`#668 <https://github.com/maliput/maliput/issues/668>`_)
* Maliput throw new errors. (`#665 <https://github.com/maliput/maliput/issues/665>`_)
* Contributors: Franco Cipollone, Santiago Lopez

1.6.0 (2025-07-29)
------------------
* Create API to obtain Geo Reference info. (`#661 <https://github.com/maliput/maliput/issues/661>`_)
* Contributors: Santiago Lopez

1.5.1 (2025-07-15)
------------------
* Always use PickGridUnit. (`#658 <https://github.com/maliput/maliput/issues/658>`_)
* Improve documentation about the LanePosition influence in the Lane::ToInertialPosition method. (`#656 <https://github.com/maliput/maliput/issues/656>`_)
* Fixes maliput_design doc. (`#654 <https://github.com/maliput/maliput/issues/654>`_)
* Fixes CI workflow. (`#653 <https://github.com/maliput/maliput/issues/653>`_)
* Contributors: Franco Cipollone, Santiago Lopez

1.5.0 (2025-05-21)
------------------
* KdTree Strategy: Register points at boundaries (`#651 <https://github.com/maliput/maliput/issues/651>`_)
* BCR: Add a maintainer (`#650 <https://github.com/maliput/maliput/issues/650>`_)
* Contributors: Franco Cipollone

1.4.2 (2025-05-14)
------------------
* Improve ComputeSampleStep impl in obj optimized generation. (`#648 <https://github.com/maliput/maliput/issues/648>`_)
* Contributors: Franco Cipollone

1.4.1 (2025-04-22)
------------------
* Bazel: Bumps eigen to 3.4.0.bcr.1.1 (`#646 <https://github.com/maliput/maliput/issues/646>`_)
* Contributors: Franco Cipollone

1.4.0 (2025-03-07)
------------------
* Add maliput::api::RoadGeometry::BackendCustomCommand. (`#643 <https://github.com/maliput/maliput/issues/643>`_)
* Extends bazel matrix for bcr_test_module. (`#642 <https://github.com/maliput/maliput/issues/642>`_)
* Contributors: Franco Cipollone, Santiago Lopez

1.3.1 (2024-10-26)
------------------
* hash: include <cstdint> (`#640 <https://github.com/maliput/maliput/issues/640>`_)
  The <cstddef> doesn't imply <cstdint> and the hashing modules use
  uint8_t which is provided by <cstdint>. This commit adds includes
  to <cstdint> to the maliput_hash.h file and the drake hash.h file
  to ensure that fixed-width integers like uint8_t are available.
* Provides another function to include route signaling in DOT graphs when adding routes (`#639 <https://github.com/maliput/maliput/issues/639>`_)
  * Provides another function to include route signaling in graphs when serialized via DOT streams.
* Adds the filter to remove routes with lane changes when RoutingConstraints::allow_lane_switch is false. (`#638 <https://github.com/maliput/maliput/issues/638>`_)
* Adds a function that allows to export a routing::graph::Graph as a dot file. (`#637 <https://github.com/maliput/maliput/issues/637>`_)
* Modifies the DistanceRouter to consume the graph and provide multi-Lane routes. (`#636 <https://github.com/maliput/maliput/issues/636>`_)
* Graph tools to perform route queries with Segments. (`#635 <https://github.com/maliput/maliput/issues/635>`_)
  * Graph tools to perform route queries with Segments.
  Router implementations will be able to use this tools such that
  they can get the set of of Lanes per Phase in Routes that connect
  point A with point B.
* Route part four - DistanceRouter (`#619 <https://github.com/maliput/maliput/issues/619>`_)
  * Provides an implementation for Router.
  - The router is based on a distance cost function.
  - Inflation of the routes to embrace lateral lanes is a TODO.
* Adds validation of end to end connectivity to Routes (`#628 <https://github.com/maliput/maliput/issues/628>`_)
* Fix typos in routing. (`#634 <https://github.com/maliput/maliput/issues/634>`_)
* Update github action versions. (`#633 <https://github.com/maliput/maliput/issues/633>`_)
* Adds bazel version to the matrix to avoid presubmit checks to fail. (`#631 <https://github.com/maliput/maliput/issues/631>`_)
  Example: https://buildkite.com/bazel/bcr-presubmit/`builds/5047#018 <https://github.com/builds/5047/issues/018>`_f2e0c-4d57-4a32-a0d4-b787aa479da9
* Fix scan_build errors. (`#632 <https://github.com/maliput/maliput/issues/632>`_)
  * Fix scan_build errors.
  * Fix format.
  ---------
* Contributors: Agustin Alba Chicar, Viktor Kronvall

1.3.0 (2024-04-29)
------------------
* Add fresnel functions (`#629 <https://github.com/ToyotaResearchInstitute/maliput/issues/629>`_)
* Contributors: Agustin Alba Chicar

1.2.0 (2024-01-03)
------------------
* [infra] templates for bcr release automation (`#621 <https://github.com/maliput/maliput/issues/622>`_)
* [infra] create a release archive with a stable url (`#621 <https://github.com/maliput/maliput/issues/621>`_)
* [bazel] yaml-cpp and tinyxml2 from the bcr instead of the fork (`#620 <https://github.com/maliput/maliput/issues/620>`_)
* Route part three (`#557 <https://github.com/maliput/maliput/issues/557>`_)
* Adds FindLaneSequence() overload that removes sequences with U-turns. (`#618 <https://github.com/maliput/maliput/issues/618>`_)
* Unifies doxygen style in favor of using @ instead of ` . (`#615 <https://github.com/maliput/maliput/issues/615>`_)
* Updates ros-tooling version to avoid error with dependency. (`#616 <https://github.com/maliput/maliput/issues/616>`_)
* Hide drake headers (`#603 <https://github.com/maliput/maliput/issues/603>`_)
* Adds buid flags as in maliput_malidrive to pair builds. (`#613 <https://github.com/maliput/maliput/issues/613>`_)
* Builds test_utilities no matter BUILD_TESTING flag. (`#612 <https://github.com/maliput/maliput/issues/612>`_)
* Adds workflow dispatch for sanitizers and scan build. (`#596 <https://github.com/maliput/maliput/issues/596>`_)
* Fix sanitizers (`#611 <https://github.com/maliput/maliput/issues/611>`_)
* Completes the section about tolerances and scale length in the design document (`#558 <https://github.com/maliput/maliput/issues/558>`_)
* Fixes bazel version not being correctly set. (`#610 <https://github.com/maliput/maliput/issues/610>`_)
* Builds test_utilities with bazel. (`#608 <https://github.com/maliput/maliput/issues/608>`_)
* Removes maliput test_utilities old compare methods (`#607 <https://github.com/maliput/maliput/issues/607>`_)
* Fixes NextPhase compare method. (`#606 <https://github.com/maliput/maliput/issues/606>`_)
* Removes gmock use from test_utilities. (`#605 <https://github.com/maliput/maliput/issues/605>`_)
* Uses new compare machinery instead of test_utilities (`#601 <https://github.com/maliput/maliput/issues/601>`_)
* Decouples test_utilities compare methods (`#600 <https://github.com/maliput/maliput/issues/600>`_)
* Excludes drake from codecov. (`#599 <https://github.com/maliput/maliput/issues/599>`_)
* Adds tests for BranchPoint and LaneEndSet api. (`#553 <https://github.com/maliput/maliput/issues/553>`_)
* Decouples routing compare methods. (`#597 <https://github.com/maliput/maliput/issues/597>`_)
* Updates configuration of codecov (`#598 <https://github.com/maliput/maliput/issues/598>`_)
* Decouples maliput types compare methods from test. (`#595 <https://github.com/maliput/maliput/issues/595>`_)
* Decouples maliput math compare methods from test. (`#592 <https://github.com/maliput/maliput/issues/592>`_)
* Remove extra macro MALIPUT_USED. (`#590 <https://github.com/maliput/maliput/issues/590>`_)
* Adds missing linking from maliput::drake::analysis to trajectories. (`#589 <https://github.com/maliput/maliput/issues/589>`_)
* Adds missing ament_export_dependencies for eigen package. (`#587 <https://github.com/maliput/maliput/issues/587>`_)
* Enables CI workflow run on main branch push. (`#588 <https://github.com/maliput/maliput/issues/588>`_)
* Fix missing bazel linkopt to experimental std++fs library (`#584 <https://github.com/maliput/maliput/issues/584>`_)
* Build on PR only, not push (`#581 <https://github.com/maliput/maliput/issues/581>`_)
* Correct codespace container selection in the devcontainer readme (`#578 <https://github.com/maliput/maliput/issues/578>`_)
* Removes code from common and systems that is unused. (`#575 <https://github.com/maliput/maliput/issues/575>`_)
* ci job for bazel build (`#577 <https://github.com/maliput/maliput/issues/577>`_)
* Local and CI Containers (`#574 <https://github.com/maliput/maliput/issues/574>`_)
* Migrate maliput_drake into maliput (`#571 <https://github.com/maliput/maliput/issues/571>`_)
* Removes fmt. (`#570 <https://github.com/maliput/maliput/issues/570>`_)
* Removes fmt from logger. (`#568 <https://github.com/maliput/maliput/issues/568>`_)
* [infra] bzlmodded (`#560 <https://github.com/maliput/maliput/issues/560>`_)
* Removes fmt from tests. (`#563 <https://github.com/maliput/maliput/issues/563>`_)
* Removes fmt from utility library. (`#567 <https://github.com/maliput/maliput/issues/567>`_)
* Implements the Route::ComputeLaneSRelation (`#555 <https://github.com/maliput/maliput/issues/555>`_)
* Removes unnecessary gflags dependency. (`#559 <https://github.com/maliput/maliput/issues/559>`_)
* Partially implements and tests Route. (`#554 <https://github.com/maliput/maliput/issues/554>`_)
* Adds Phase implementation and test. (`#550 <https://github.com/maliput/maliput/issues/550>`_)
* Adds codecov. (`#552 <https://github.com/maliput/maliput/issues/552>`_)
* Routing constraints implementation (`#549 <https://github.com/maliput/maliput/issues/549>`_)
* [Routing] Initial public API proposal. (`#546 <https://github.com/maliput/maliput/issues/546>`_)
  Co-authored-by: Franco Cipollone <53065142+francocipollone@users.noreply.github.com>
* Contributors: Agustin Alba Chicar, Daniel Stonier, Franco Cipollone

1.1.1 (2023-03-13)
------------------
* Provides LaneSRange::GetIntersection method. (`#542 <https://github.com/maliput/maliput/issues/542>`_)
* Contributors: Franco Cipollone

1.1.0 (2023-02-08)
------------------
* Adds dimension static const to vector class. (`#541 <https://github.com/maliput/maliput/issues/541>`_)
* Fixes logger-level-0ff behavior. (`#540 <https://github.com/maliput/maliput/issues/540>`_)
* Adds maliput profiler (`#538 <https://github.com/maliput/maliput/issues/538>`_)
* Provides a default ToRoadPosition/FindRoadPosition implementations using kdtree data structure (`#517 <https://github.com/maliput/maliput/issues/517>`_)
* PhaseRingBookLoader supporting empty rules for the phases. (`#536 <https://github.com/maliput/maliput/issues/536>`_)
* Provides new-rule-api compatible RoadNetwork's constructor. (`#535 <https://github.com/maliput/maliput/issues/535>`_)
* Contributors: Franco Cipollone

1.0.9 (2022-11-28)
------------------
* Provides default populated discrete and range value rule state providers. (`#533 <https://github.com/maliput/maliput/issues/533>`_)
* Adds a default populated ManualPhaseProvider. (`#530 <https://github.com/maliput/maliput/issues/530>`_)
* Maliput Plugin: Adds interface for providing default parameters from the backends (`#532 <https://github.com/maliput/maliput/issues/532>`_)
* Contributors: Franco Cipollone

1.0.8 (2022-11-10)
------------------
* Brings range validator from maliput_malidrive. (`#529 <https://github.com/maliput/maliput/issues/529>`_)
* Update triage.yml (`#526 <https://github.com/maliput/maliput/issues/526>`_)
* Adds convenient test utility method. (`#525 <https://github.com/maliput/maliput/issues/525>`_)
* Adds a test function for LaneEnds. (`#524 <https://github.com/maliput/maliput/issues/524>`_)
* Adds IsLanePositionResultClose macro. (`#522 <https://github.com/maliput/maliput/issues/522>`_)
* Contributors: Agustin Alba Chicar, Franco Cipollone

1.0.7 (2022-09-14)
------------------
* Modifies ToLanePosition and adds ToSegmentPosition. (`#521 <https://github.com/maliput/maliput/issues/521>`_)
* Contributors: Franco Cipollone

1.0.6 (2022-08-16)
------------------
* Implements KDTree::RangeSearch method. (`#520 <https://github.com/maliput/maliput/issues/520>`_)
* Adds AxisAlignedBox. (`#519 <https://github.com/maliput/maliput/issues/519>`_)
* Brings BoundingRegion's related stuff from maliput_object. (`#518 <https://github.com/maliput/maliput/issues/518>`_)
* Adds KDTree to maliput::math. (`#515 <https://github.com/maliput/maliput/issues/515>`_)
* Contributors: Franco Cipollone

1.0.5 (2022-07-26)
------------------
* Provides convenient method for loading a RN via plugins. (`#512 <https://github.com/maliput/maliput/issues/512>`_)
* Adds triage workflow. (`#513 <https://github.com/maliput/maliput/issues/513>`_)
* Improves README. (`#511 <https://github.com/maliput/maliput/issues/511>`_)
* Update README.md with new github.com/maliput URLs (`#510 <https://github.com/maliput/maliput/issues/510>`_)
  Needed due to the transition to the "maliput" organization.
  Also refer developers to new documentation website.
* Contributors: Chien-Liang Fok, Franco Cipollone

1.0.4 (2022-06-13)
------------------
* Fixes include folder installation. (`#508 <https://github.com/maliput/maliput/issues/508>`_)
* Uses ros-action-ci in build.yaml workflow. (`#505 <https://github.com/maliput/maliput/issues/505>`_)
* Contributors: Franco Cipollone

1.0.3 (2022-06-08)
------------------
* moving maliput to the root (`#506 <https://github.com/maliput/maliput/issues/506>`_)
* Contributors: Tully Foote

1.0.2 (2022-06-06)
------------------

* Preparing for binary release

1.0.1 (2022-06-02)
------------------

* Preparing for binary release

  1.0.0 (2021-0X-XX)
------------------

* First official release
