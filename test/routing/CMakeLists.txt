ament_add_gmock(route_test route_test.cc)
ament_add_gmock(route_phase_test route_phase_test.cc)
ament_add_gtest(routing_constraints_test routing_constraints_test.cc)

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
          maliput::routing
          maliput::test_utilities
      )

    endif()
endmacro()

add_dependencies_to_test(route_phase_test)
add_dependencies_to_test(route_test)
add_dependencies_to_test(routing_constraints_test)
