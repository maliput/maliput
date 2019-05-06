set(TEST_UTILS_SOURCES
  multilane_brute_force_integral.cc
  multilane_types_compare.cc)

add_library(test_utilities_multilane ${TEST_UTILS_SOURCES})

target_include_directories(
  test_utilities_multilane
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>;${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(test_utilities_multilane
  multilane
  maliput::maliput_test_utilities
  drake::drake)

install(
  TARGETS test_utilities_multilane
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(test_utilities_multilane)
