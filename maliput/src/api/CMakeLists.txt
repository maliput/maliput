##############################################################################
# Sources
##############################################################################

set(API_SOURCES
  basic_id_index.cc
  intersection.cc
  lane.cc
  lane_data.cc
  road_geometry.cc
  road_network.cc
  road_network_validator.cc
  rules/direction_usage_rule.cc
  rules/phase.cc
  rules/phase_ring.cc
  rules/traffic_lights.cc)

add_library(api ${API_SOURCES})

target_include_directories(api
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(api
  PUBLIC
    common
    drake::drake
)
target_link_libraries(api
  PUBLIC
    common
    drake::drake)

# ament_target_dependencies(api
#   "drake"
# )

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
