##############################################################################
# Sources
##############################################################################

set(SOURCES
  generate_obj.cc
  generate_string.cc
  generate_urdf.cc
  mesh_simplification.cc
  segment_analysis.cc)

add_library(utilities ${SOURCES})

target_include_directories(utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(utilities
  PRIVATE
    fmt::fmt
  PUBLIC
    api
    common
    math
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS utilities
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(utilities)
