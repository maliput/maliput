ament_add_gmock(brute_force_find_road_positions_test brute_force_find_road_positions_test.cc)
ament_add_gtest(filter_positions_test filter_positions_test.cc)
ament_add_gtest(geometry_base_test geometry_base_test.cc)

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
          maliput::geometry_base
          maliput::test_utilities
      )

    endif()
endmacro()

add_dependencies_to_test(brute_force_find_road_positions_test)
add_dependencies_to_test(filter_positions_test)
add_dependencies_to_test(geometry_base_test)
