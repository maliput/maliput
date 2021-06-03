##############################################################################
# Sources
##############################################################################

set(SOURCES
  generate_obj.cc
  generate_string.cc
  generate_urdf.cc
  mesh_simplification.cc
  segment_analysis.cc
  thread_pool.cc)

add_library(utilities ${SOURCES})

add_library(maliput::utilities ALIAS utilities)

set_target_properties(utilities
  PROPERTIES
    OUTPUT_NAME maliput_utilities
)

target_include_directories(utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(utilities
  PRIVATE
    fmt::fmt
  PUBLIC
    maliput::api
    maliput::common
    maliput::math
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS utilities
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(utilities)
