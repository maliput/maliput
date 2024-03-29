add_library(drake_systems_framework
  abstract_value_cloner.cc
  abstract_values.cc
  basic_vector.cc
  cache.cc
  cache_entry.cc
  context_base.cc
  context.cc
  continuous_state.cc
  dependency_tracker.cc
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
  state.cc
  subvector.cc
  system_base.cc
  system.cc
  system_constraint.cc
  system_output.cc
  system_scalar_converter.cc
  system_type_tag.cc
  system_visitor.cc
  value_producer.cc
  vector_base.cc
  witness_function.cc
)

add_library(maliput::drake_systems_framework ALIAS drake_systems_framework)

set_target_properties(
  drake_systems_framework
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_framework
)

target_include_directories(
  drake_systems_framework
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  drake_systems_framework
    Eigen3::Eigen
    maliput::drake_common
)

install(
  TARGETS drake_systems_framework
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
