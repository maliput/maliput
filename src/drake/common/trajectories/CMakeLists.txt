add_library(drake_common_trajectories
  piecewise_polynomial.cc
  piecewise_trajectory.cc
  trajectory.cc
)

add_library(maliput::drake_common_trajectories ALIAS drake_common_trajectories)

set_target_properties(
  drake_common_trajectories
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_common_trajectories
)

target_include_directories(
  drake_common_trajectories
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  drake_common_trajectories
    Eigen3::Eigen
    maliput::drake_common
)

install(
  TARGETS drake_common_trajectories
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
