ament_add_gtest(2x2_intersection_test 2x2_intersection_test.cc)
ament_add_gtest(multilane_arc_road_curve_test multilane_arc_road_curve_test.cc)
ament_add_gtest(multilane_lanes_test multilane_lanes_test.cc)
ament_add_gtest(multilane_line_road_curve_test multilane_line_road_curve_test.cc)
ament_add_gtest(multilane_road_curve_accuracy_test multilane_road_curve_accuracy_test.cc TIMEOUT 120)
ament_add_gtest(multilane_road_geometry_test multilane_road_geometry_test.cc)
ament_add_gtest(multilane_segments_test multilane_segments_test.cc)

ament_add_gmock(multilane_builder_test multilane_builder_test.cc)
ament_add_gmock(multilane_connection_test multilane_connection_test.cc)
ament_add_gmock(multilane_loader_test multilane_loader_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
      )

      ament_target_dependencies(${target}
          "maliput"
          "drake"
      )

      target_link_libraries(${target}
          maliput::api
          maliput::common
          multilane_test_utilities
          drake::drake)

    endif()
endmacro()

add_dependencies_to_test(2x2_intersection_test)
add_dependencies_to_test(multilane_arc_road_curve_test)
add_dependencies_to_test(multilane_builder_test)
add_dependencies_to_test(multilane_connection_test)
add_dependencies_to_test(multilane_lanes_test)
add_dependencies_to_test(multilane_line_road_curve_test)
add_dependencies_to_test(multilane_loader_test)
add_dependencies_to_test(multilane_road_curve_accuracy_test)
add_dependencies_to_test(multilane_road_geometry_test)
add_dependencies_to_test(multilane_segments_test)
