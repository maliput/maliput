ament_add_gtest(intersection_test intersection_test.cc)
ament_add_gtest(manual_phase_provider_test manual_phase_provider_test.cc)
ament_add_gtest(manual_discrete_value_rule_state_provider_test manual_discrete_value_rule_state_provider_test.cc)
ament_add_gtest(manual_range_value_rule_state_provider_test manual_range_value_rule_state_provider_test.cc)
ament_add_gtest(manual_right_of_way_rule_state_provider_test manual_right_of_way_rule_state_provider_test)
ament_add_gtest(manual_phase_ring_book_test manual_phase_ring_book_test.cc)
ament_add_gtest(manual_rulebook_test manual_rulebook_test.cc)
ament_add_gtest(phase_based_right_of_way_rule_state_provider_test phase_based_right_of_way_rule_state_provider_test.cc)
ament_add_gtest(traffic_light_book_test traffic_light_book_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      ament_target_dependencies(${target}
          "drake"
      )

      target_link_libraries(${target}
          api
          base
          maliput_test_utilities
          drake::drake)

    endif()
endmacro()

add_dependencies_to_test(intersection_test)
add_dependencies_to_test(manual_phase_provider_test)
add_dependencies_to_test(manual_discrete_value_rule_state_provider_test)
add_dependencies_to_test(manual_range_value_rule_state_provider_test)
add_dependencies_to_test(manual_right_of_way_rule_state_provider_test)
add_dependencies_to_test(manual_phase_ring_book_test)
add_dependencies_to_test(manual_rulebook_test)
add_dependencies_to_test(phase_based_right_of_way_rule_state_provider_test)
add_dependencies_to_test(traffic_light_book_test)
