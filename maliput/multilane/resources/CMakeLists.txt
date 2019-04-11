##############################################################################
# Resources
##############################################################################

set(MULTILANE_RESOURCES
  2x2_intersection.yaml
  circuit.yaml
  double_ring.yaml
  fig8.yaml
  helix.yaml
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
