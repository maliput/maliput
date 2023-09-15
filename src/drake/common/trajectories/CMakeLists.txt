add_library(trajectories
  piecewise_polynomial.cc
  piecewise_trajectory.cc
  trajectory.cc
)

add_library(maliput_drake::trajectories ALIAS trajectories)

set_target_properties(
  trajectories
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_trajectories
)

target_include_directories(
  trajectories
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  trajectories
    Eigen3::Eigen
    fmt::fmt
    maliput_drake::common
)

install(
  TARGETS trajectories
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
