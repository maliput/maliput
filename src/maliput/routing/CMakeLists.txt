##############################################################################
# Sources
##############################################################################

set(ROUTING_SOURCES
  compare.cc
  derive_lane_s_routes.cc
  find_lane_sequences.cc
  graph/graph.cc
  graph/graph_utils.cc
  phase.cc
  route.cc
  routing_constraints.cc
)

add_library(routing ${ROUTING_SOURCES})

add_library(maliput::routing ALIAS routing)

set_target_properties(routing
  PROPERTIES
    OUTPUT_NAME maliput_routing
)

target_include_directories(routing
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(routing
  PUBLIC
    maliput::api
    maliput::common
  PRIVATE
    maliput::geometry_base
)

if(MALIPUT_PROFILER_ENABLE)
  target_link_libraries(routing
    PRIVATE
      ignition-common3::profiler
  )
endif()

##############################################################################
# Export
##############################################################################

install(
  TARGETS routing
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
