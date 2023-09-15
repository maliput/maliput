add_library(analysis
  antiderivative_function.cc
  dense_output.cc
  hermitian_dense_output.cc
  initial_value_problem.cc
  integrator_base.cc
  runge_kutta3_integrator.cc
  scalar_dense_output.cc
  scalar_initial_value_problem.cc
  scalar_view_dense_output.cc
  stepwise_dense_output.cc
)

add_library(maliput_drake::analysis ALIAS analysis)

set_target_properties(
  analysis
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_analysis
)

target_include_directories(
  analysis
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  analysis
    Eigen3::Eigen
    fmt::fmt
    maliput_drake::common
    maliput_drake::framework
)

install(
  TARGETS analysis
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
