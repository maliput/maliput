ament_add_gtest(discrete_value_rule_state_provider_test discrete_value_rule_state_provider_test.cc)
ament_add_gmock(lane_test lane_test.cc)
ament_add_gtest(lane_data_test lane_data_test.cc)
ament_add_gtest(maliput_test maliput_test.cc)
ament_add_gtest(phase_ring_test phase_ring_test.cc)
ament_add_gtest(phase_test phase_test.cc)
ament_add_gtest(range_value_rule_state_provider_test range_value_rule_state_provider_test.cc)
ament_add_gtest(regions_test regions_test.cc)
ament_add_gtest(right_of_way_rule_state_provider_test right_of_way_rule_state_provider_test.cc)
ament_add_gtest(road_network_test road_network_test.cc)
ament_add_gtest(road_network_validator_test road_network_validator_test.cc)
ament_add_gtest(rule_registry_test rule_registry_test.cc)
ament_add_gtest(rule_test rule_test.cc)
ament_add_gtest(rules_direction_usage_test rules_direction_usage_test.cc)
ament_add_gtest(rules_right_of_way_test rules_right_of_way_test.cc)
ament_add_gtest(rules_road_rulebook_test rules_road_rulebook_test.cc)
ament_add_gtest(rules_speed_limit_test rules_speed_limit_test.cc)
ament_add_gtest(traffic_lights_test traffic_lights_test.cc)
ament_add_gtest(type_specific_identifier_test type_specific_identifier_test.cc)
ament_add_gtest(unique_id_test unique_id_test.cc)

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
          maliput::common
          maliput::math
          maliput::test_utilities
      )

    endif()
endmacro()

add_dependencies_to_test(discrete_value_rule_state_provider_test)
add_dependencies_to_test(lane_test)
add_dependencies_to_test(lane_data_test)
add_dependencies_to_test(maliput_test)
add_dependencies_to_test(phase_ring_test)
add_dependencies_to_test(phase_test)
add_dependencies_to_test(range_value_rule_state_provider_test)
add_dependencies_to_test(regions_test)
add_dependencies_to_test(right_of_way_rule_state_provider_test)
add_dependencies_to_test(road_network_test)
add_dependencies_to_test(road_network_validator_test)
add_dependencies_to_test(rule_registry_test)
add_dependencies_to_test(rule_test)
add_dependencies_to_test(rules_direction_usage_test)
add_dependencies_to_test(rules_right_of_way_test)
add_dependencies_to_test(rules_road_rulebook_test)
add_dependencies_to_test(rules_speed_limit_test)
add_dependencies_to_test(traffic_lights_test)
add_dependencies_to_test(type_specific_identifier_test)
add_dependencies_to_test(unique_id_test)
