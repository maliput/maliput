ament_add_gtest(base_rule_registry_test rule_registry_test.cc)
ament_add_gtest(intersection_test intersection_test.cc)
ament_add_gtest(intersection_book_test intersection_book_test.cc)
ament_add_gtest(manual_phase_provider_test manual_phase_provider_test.cc)
ament_add_gtest(manual_discrete_value_rule_state_provider_test manual_discrete_value_rule_state_provider_test.cc)
ament_add_gtest(manual_range_value_rule_state_provider_test manual_range_value_rule_state_provider_test.cc)
ament_add_gtest(manual_right_of_way_rule_state_provider_test manual_right_of_way_rule_state_provider_test)
ament_add_gtest(manual_phase_ring_book_test manual_phase_ring_book_test.cc)
ament_add_gtest(manual_rulebook_test manual_rulebook_test.cc)
ament_add_gtest(phased_discrete_rule_state_provider_test phased_discrete_rule_state_provider_test.cc)
ament_add_gtest(phase_based_right_of_way_rule_state_provider_test phase_based_right_of_way_rule_state_provider_test.cc)
ament_add_gtest(phase_ring_book_loader_test phase_ring_book_loader_test.cc)
ament_add_gtest(rule_filter_test rule_filter_test.cc)
ament_add_gtest(rule_tools_test rule_tools_test.cc)
ament_add_gtest(rule_registry_loader_test rule_registry_loader_test.cc)
ament_add_gtest(traffic_light_book_test traffic_light_book_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::api
          maliput::base
          fmt::fmt
          test_utilities
      )

    endif()
endmacro()

add_dependencies_to_test(base_rule_registry_test)
add_dependencies_to_test(intersection_test)
add_dependencies_to_test(intersection_book_test)
add_dependencies_to_test(manual_phase_provider_test)
add_dependencies_to_test(manual_discrete_value_rule_state_provider_test)
add_dependencies_to_test(manual_range_value_rule_state_provider_test)
add_dependencies_to_test(manual_right_of_way_rule_state_provider_test)
add_dependencies_to_test(manual_phase_ring_book_test)
add_dependencies_to_test(manual_rulebook_test)
add_dependencies_to_test(phased_discrete_rule_state_provider_test)
add_dependencies_to_test(phase_based_right_of_way_rule_state_provider_test)
add_dependencies_to_test(phase_ring_book_loader_test)
add_dependencies_to_test(rule_filter_test)
add_dependencies_to_test(rule_tools_test)
add_dependencies_to_test(rule_registry_loader_test)
add_dependencies_to_test(traffic_light_book_test)
