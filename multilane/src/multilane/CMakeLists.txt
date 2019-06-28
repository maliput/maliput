##############################################################################
# Sources
##############################################################################

set(MULTILANE_SOURCES
  arc_road_curve.cc
  branch_point.cc
  builder.cc
  connection.cc
  cubic_polynomial.cc
  junction.cc
  lane.cc
  line_road_curve.cc
  loader.cc
  multilane_onramp_merge.cc
  road_curve.cc
  road_geometry.cc
  segment.cc)

add_library(multilane ${MULTILANE_SOURCES})

target_include_directories(multilane
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

ament_target_dependencies(multilane
  "maliput"
  "drake"
)

target_link_libraries(multilane
  maliput::api
  drake::drake
  yaml-cpp
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS multilane
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(multilane)
