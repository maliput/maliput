##############################################################################
# Sources
##############################################################################

set(TEST_UTILS_SOURCES
  check_id_indexing.cc
  maliput_types_compare.cc
  mock_geometry.cc
  rules_direction_usage_compare.cc
  rules_right_of_way_compare.cc
  rules_speed_limit_compare.cc
  mock.cc)

add_library(test_utilities ${TEST_UTILS_SOURCES})

target_include_directories(
  test_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_target_dependencies(test_utilities
  "drake"
)

target_link_libraries(test_utilities
  api
  geometry_base
  drake::drake)

install(
    TARGETS test_utilities EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(test_utilities)
