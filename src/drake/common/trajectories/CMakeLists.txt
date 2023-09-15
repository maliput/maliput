add_library(trajectories
  piecewise_polynomial.cc
  piecewise_trajectory.cc
  trajectory.cc
)

add_library(maliput_drake::trajectories ALIAS drake-common-trajectories)

set_target_properties(
  drake-common-trajectories
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_trajectories
)

target_include_directories(
  drake-common-trajectories
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  drake-common-trajectories
    Eigen3::Eigen
    maliput_drake::common
)

install(
  TARGETS drake-common-trajectories
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
