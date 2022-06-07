##############################################################################
# Sources
##############################################################################

set(BASE_SOURCES
  intersection.cc
  intersection_book.cc
  intersection_book_loader.cc
  manual_discrete_value_rule_state_provider.cc
  manual_phase_provider.cc
  manual_range_value_rule_state_provider.cc
  manual_right_of_way_rule_state_provider.cc
  manual_phase_ring_book.cc
  manual_rulebook.cc
  phased_discrete_rule_state_provider.cc
  phase_based_right_of_way_rule_state_provider.cc
  phase_ring_book_loader.cc
  phase_ring_book_loader_old_rules.cc
  road_rulebook_loader.cc
  road_rulebook_loader_using_rule_registry.cc
  rule_filter.cc
  rule_registry.cc
  rule_registry_loader.cc
  rule_tools.cc
  traffic_light_book.cc
  traffic_light_book_loader.cc
  yaml_conversion.cc
)

add_library(base ${BASE_SOURCES})

add_library(maliput::base ALIAS base)

set_target_properties(base
  PROPERTIES
    OUTPUT_NAME maliput_base
)

target_include_directories(base
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(base
  PUBLIC
    maliput::api
    maliput::common
    yaml-cpp
)

##############################################################################
# Tests
##############################################################################

##############################################################################
# Export
##############################################################################

include(CMakePackageConfigHelpers)

install(
  TARGETS base
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
