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

add_library(maliput_drake::analysis ALIAS drake-systems-analysis)

set_target_properties(
  drake-systems-analysis
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_analysis
)

target_include_directories(
  drake-systems-analysis
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  drake-systems-analysis
    Eigen3::Eigen
    maliput_drake::common
    maliput_drake::framework
)

install(
  TARGETS drake-systems-analysis
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
