##############################################################################
# Sources
##############################################################################
set(SOURCES
    branch_point.cc
    brute_force_find_road_positions_strategy.cc
    filter_positions.cc
    junction.cc
    lane.cc
    road_geometry.cc
    segment.cc)

add_library(geometry_base ${SOURCES})

add_library(maliput::geometry_base ALIAS geometry_base)

set_target_properties(geometry_base
  PROPERTIES
    OUTPUT_NAME maliput_geometry_base
)

target_include_directories(geometry_base
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(geometry_base
  PUBLIC
    maliput::api
)

##############################################################################
# Tests
##############################################################################

##############################################################################
# Export
##############################################################################

install(
  TARGETS geometry_base
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
