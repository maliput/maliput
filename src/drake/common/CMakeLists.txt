add_library(drake_common
  cond.cc
  double_overloads.cc
  drake_assert_and_throw.cc
  hash.cc
  nice_type_name.cc
  nice_type_name_override.cc
  pointer_cast.cc
  polynomial.cc
  random.cc
  sorted_pair.cc
  text_logging.cc
  value.cc
)

add_library(maliput::drake_common ALIAS drake_common)

set_target_properties(
  drake_common
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_drake_common
)

target_include_directories(
  drake_common
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  drake_common
    dl
    Eigen3::Eigen
)

install(
  TARGETS drake_common
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

add_subdirectory(trajectories)