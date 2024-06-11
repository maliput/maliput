ament_add_gmock(derive_lane_s_routes_test derive_lane_s_routes_test.cc)
ament_add_gmock(route_test route_test.cc)
ament_add_gtest(routing_compare_test compare_test.cc)
ament_add_gmock(routing_phase_test phase_test.cc)
ament_add_gtest(routing_constraints_test routing_constraints_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::api
          maliput::common
          maliput::routing
          maliput::test_utilities
      )

    endif()
endmacro()

add_dependencies_to_test(derive_lane_s_routes_test)
add_dependencies_to_test(route_test)
add_dependencies_to_test(routing_compare_test)
add_dependencies_to_test(routing_phase_test)
add_dependencies_to_test(routing_constraints_test)
