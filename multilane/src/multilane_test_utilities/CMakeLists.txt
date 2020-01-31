set(TEST_UTILS_SOURCES
  fixtures.cc
  multilane_brute_force_integral.cc
  multilane_types_compare.cc)

add_library(multilane_test_utilities ${TEST_UTILS_SOURCES})

find_package(ament_cmake_gmock REQUIRED)

ament_find_gmock()

target_include_directories(
  multilane_test_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${GMOCK_INCLUDE_DIRS}
)

target_link_libraries(multilane_test_utilities
  drake::drake
  maliput::api
  maliput::common
  maliput::maliput_test_utilities
  maliput::math
  multilane
)

install(
    TARGETS multilane_test_utilities
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(multilane_test_utilities)
