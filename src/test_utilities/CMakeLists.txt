##############################################################################
# Sources
##############################################################################
if(BUILD_TESTING)


set(TEST_UTILS_SOURCES
  mock.cc
  mock_geometry.cc
)

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
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(test_utilities
  maliput::api
  maliput::common
  maliput::geometry_base
  maliput::math
)

install(
  TARGETS test_utilities
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

endif()
