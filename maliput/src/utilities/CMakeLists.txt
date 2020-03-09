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

ament_target_dependencies(maliput-utilities
  PUBLIC
    "maliput"
)

target_link_libraries(maliput-utilities
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
    TARGETS maliput-utilities
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(maliput-utilities)
