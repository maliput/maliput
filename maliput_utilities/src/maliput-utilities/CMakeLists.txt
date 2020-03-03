##############################################################################
# Sources
##############################################################################

set(SOURCES
  generate_obj.cc
  generate_string.cc
  generate_urdf.cc
  mesh_simplification.cc
  segment_analysis.cc)

add_library(maliput-utilities ${SOURCES})

target_include_directories(maliput-utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(maliput-utilities
  PRIVATE
    fmt::fmt
  PUBLIC
    maliput::api
    maliput::common
    maliput::math
    multilane::multilane
)

add_executable(yaml_to_obj
  yaml_to_obj.cc)

ament_target_dependencies(yaml_to_obj
  "multilane"
  "maliput"
)

target_link_libraries(yaml_to_obj
    yaml-cpp
    gflags
    multilane::multilane
    maliput-utilities
    drake::drake
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS maliput-utilities
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

install(TARGETS yaml_to_obj
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_libraries(maliput-utilities)
