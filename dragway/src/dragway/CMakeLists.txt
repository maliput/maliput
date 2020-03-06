##############################################################################
# Sources
##############################################################################

set(SOURCES
  branch_point.cc
  junction.cc
  lane.cc
  road_geometry.cc
  segment.cc)

add_library(dragway ${SOURCES})

target_include_directories(dragway
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(dragway
  maliput::api
  maliput::common
  maliput::geometry_base
)

ament_target_dependencies(dragway
    "maliput"
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS dragway
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(dragway)
