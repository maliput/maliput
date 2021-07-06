find_package(ament_cmake_gtest REQUIRED)

ament_find_gtest()

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

add_library(test_utilities ${TEST_UTILS_SOURCES})

add_library(maliput::test_utilities ALIAS test_utilities)

set_target_properties(test_utilities
  PROPERTIES
    OUTPUT_NAME maliput_test_utilities
)

target_include_directories(
  test_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GTEST_INCLUDE_DIRS}
)

target_link_libraries(test_utilities
 PUBLIC
  maliput::api
  maliput::common
  maliput::math
  maliput::geometry_base
 PRIVATE
  ${GTEST_LIBRARIES}
)

install(
  TARGETS test_utilities
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(test_utilities)
