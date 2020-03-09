##############################################################################
# Sources
##############################################################################

set(TEST_UTILS_SOURCES
  check_id_indexing.cc
  maliput_math_compare.cc
  maliput_types_compare.cc
  mock.cc
  mock_geometry.cc
  phases_compare.cc
  rules_compare.cc
  rules_direction_usage_compare.cc
  rules_right_of_way_compare.cc
  rules_speed_limit_compare.cc
  traffic_lights_compare.cc)

add_library(maliput_test_utilities ${TEST_UTILS_SOURCES})

target_include_directories(
  maliput_test_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(maliput_test_utilities
  api
  common
  math
  geometry_base
)

install(
    TARGETS maliput_test_utilities EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(maliput_test_utilities)
