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

target_include_directories(api
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(api
  PUBLIC
    common
    math
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS api
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(api)
