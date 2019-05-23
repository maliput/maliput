##############################################################################
# Resources
##############################################################################

set(MULTILANE_RESOURCES
  2x2_intersection.yaml
  branch_and_merge.yaml
  circuit.yaml
  double_ring.yaml
  dual_non_intersecting_lanes.yaml
  fig8.yaml
  helix.yaml
  long_start_and_end_lanes.yaml
  loop.yaml
  multi_branch.yaml
  ring.yaml
  sidewinder.yaml
  tee_intersection.yaml
  village.yaml)

##############################################################################
# Export
##############################################################################

install(
  FILES
    ${MULTILANE_RESOURCES}
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)
