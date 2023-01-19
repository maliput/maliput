##############################################################################
# Sources
##############################################################################

set(API_SOURCES
  basic_id_index.cc
  intersection.cc
  lane.cc
  lane_data.cc
  regions.cc
  road_geometry.cc
  road_network.cc
  road_network_validator.cc
  rules/direction_usage_rule.cc
  rules/discrete_value_rule.cc
  rules/phase.cc
  rules/phase_ring.cc
  rules/range_value_rule.cc
  rules/rule.cc
  rules/rule_registry.cc
  rules/traffic_lights.cc)

add_library(api ${API_SOURCES})

add_library(maliput::api ALIAS api)

set_target_properties(api
  PROPERTIES
    OUTPUT_NAME maliput_api
)

target_include_directories(api
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(api
  PUBLIC
    maliput::common
    maliput::math
)

if(MALIPUT_PROFILER_ENABLE)
  target_link_libraries(api
    PRIVATE
      ignition-common3::profiler
  )
endif()

##############################################################################
# Export
##############################################################################

install(
  TARGETS api
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
