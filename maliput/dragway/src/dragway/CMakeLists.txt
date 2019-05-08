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

ament_target_dependencies(dragway
    "maliput-utilities"
    "maliput"
    "drake"
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

add_executable(dragway_to_urdf
  dragway_to_urdf.cc
)

target_link_libraries(dragway_to_urdf
  PRIVATE
    dragway
    gflags
    maliput-utilities::maliput-utilities
    maliput::common
)

install(TARGETS dragway_to_urdf
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_libraries(dragway)
