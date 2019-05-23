set(TEST_UTILS_SOURCES
  fixtures.cc
  multilane_brute_force_integral.cc
  multilane_types_compare.cc)

add_library(multilane_test_utilities ${TEST_UTILS_SOURCES})

target_include_directories(
  multilane_test_utilities
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(multilane_test_utilities
  multilane
  maliput::api
  maliput::common
  maliput::maliput_test_utilities
  drake::drake)

install(
    TARGETS multilane_test_utilities
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(multilane_test_utilities)
