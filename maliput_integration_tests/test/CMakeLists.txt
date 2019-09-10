find_package(ament_cmake_gtest REQUIRED)

macro(add_dependencies_to_test target)
    if (TARGET ${target})
      ament_target_dependencies(${target}
        "maliput"
        "multilane"
        "dragway"
        "drake"
      )
      target_link_libraries(${target}
        multilane::multilane
        multilane::multilane_test_utilities
        maliput::api
        maliput::maliput_test_utilities
        maliput::base
        maliput::common
        maliput::routing
        dragway::dragway
        dragway::dragway_test_utilities
        drake::drake)
    endif()
endmacro()

ament_add_gtest(find_lane_sequences_test find_lane_sequences_test.cc)
ament_add_gtest(derive_lane_s_routes_test derive_lane_s_routes_test.cc)
ament_add_gtest(intersection_book_loader_test intersection_book_loader_test.cc)
ament_add_gtest(phase_ring_book_loader_test phase_ring_book_loader_test.cc)
ament_add_gtest(road_rulebook_loader_test road_rulebook_loader_test.cc)
ament_add_gtest(traffic_light_book_loader_test traffic_light_book_loader_test.cc)
ament_add_gtest(waypoints_test waypoints_test.cc)

add_dependencies_to_test(find_lane_sequences_test)
add_dependencies_to_test(derive_lane_s_routes_test)
add_dependencies_to_test(intersection_book_loader_test)
add_dependencies_to_test(phase_ring_book_loader_test)
add_dependencies_to_test(road_rulebook_loader_test)
add_dependencies_to_test(traffic_light_book_loader_test)
add_dependencies_to_test(waypoints_test)
