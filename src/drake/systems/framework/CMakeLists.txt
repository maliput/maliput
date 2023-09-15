add_library(framework
  abstract_value_cloner.cc
  abstract_values.cc
  basic_vector.cc
  cache.cc
  cache_entry.cc
  context_base.cc
  context.cc
  continuous_state.cc
  dependency_tracker.cc
  diagram_builder.cc
  diagram.cc
  diagram_context.cc
  diagram_continuous_state.cc
  diagram_discrete_values.cc
  diagram_output_port.cc
  diagram_state.cc
  discrete_values.cc
  event_collection.cc
  fixed_input_port_value.cc
  input_port_base.cc
  input_port.cc
  leaf_context.cc
  leaf_output_port.cc
  leaf_system.cc
  model_values.cc
  output_port_base.cc
  output_port.cc
  parameters.cc
  port_base.cc
  scalar_conversion_traits.cc
  single_output_vector_source.cc
  state.cc
  subvector.cc
  supervector.cc
  system_base.cc
  system.cc
  system_constraint.cc
  system_html.cc
  system_output.cc
  system_scalar_converter.cc
  system_symbolic_inspector.cc
  system_type_tag.cc
  system_visitor.cc
  value_producer.cc
  vector_base.cc
  vector_system.cc
  witness_function.cc
)

add_library(maliput_drake::framework ALIAS framework)

set_target_properties(
  framework
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_framework
)

target_include_directories(
  framework
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  framework
    fmt::fmt
    Eigen3::Eigen
    maliput_drake::common
)

install(
  TARGETS framework
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
