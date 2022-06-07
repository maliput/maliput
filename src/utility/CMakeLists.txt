##############################################################################
# Sources
##############################################################################

set(SOURCES
  file_utils.cc
  generate_obj.cc
  generate_string.cc
  generate_urdf.cc
  mesh_simplification.cc
  segment_analysis.cc
  thread_pool.cc
)
add_library(utility ${SOURCES})

add_library(maliput::utility ALIAS utility)

set_target_properties(utility
  PROPERTIES
    OUTPUT_NAME maliput_utility
)
target_link_libraries(utility
  PRIVATE
    fmt::fmt
  PUBLIC
    stdc++fs
    maliput::api
    maliput::common
    maliput::math
)

target_include_directories(utility
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

##############################################################################
# Export
##############################################################################

install(
  TARGETS utility
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
