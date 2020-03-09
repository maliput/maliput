##############################################################################
# Sources
##############################################################################

set(ROUTING_SOURCES
  derive_lane_s_routes.cc
  find_lane_sequences.cc)

add_library(routing ${ROUTING_SOURCES})

target_include_directories(routing
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(routing
  PUBLIC
    api
    common
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS routing
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

ament_export_libraries(routing)
