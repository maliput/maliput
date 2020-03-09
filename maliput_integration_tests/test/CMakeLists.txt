find_package(ament_cmake_gtest REQUIRED)

macro(add_dependencies_to_test target)
    if (TARGET ${target})
      ament_target_dependencies(${target}
        "dragway"
        "drake"
        "maliput"
        "multilane"
      )
      target_link_libraries(${target}
        dragway::dragway
        dragway::dragway_test_utilities
        drake::drake
        fmt::fmt
        maliput::api
        maliput::base
        maliput::common
        maliput::maliput_test_utilities
        maliput::routing
        maliput::utilities
        multilane::multilane
        multilane::multilane_test_utilities)
    endif()
endmacro()

set(TEST_PATH ${CMAKE_CURRENT_BINARY_DIR})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/MtlContent.mtl DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/ChangeOrigin.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/DefaultObjContent.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/HighlightedSegments.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/NoBranchPointsObjContent.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/StackedBranchPointsObjContent.obj DESTINATION ${TEST_PATH})

ament_add_gtest(derive_lane_s_routes_test derive_lane_s_routes_test.cc)
ament_add_gtest(find_lane_sequences_test find_lane_sequences_test.cc)
ament_add_gtest(generate_obj_test generate_obj_test.cc WORKING_DIRECTORY ${TEST_PATH})
ament_add_gtest(generate_string_test generate_string_test.cc)
ament_add_gtest(generate_urdf_test generate_urdf_test.cc)
ament_add_gtest(intersection_book_loader_test intersection_book_loader_test.cc)
ament_add_gtest(phase_ring_book_loader_test phase_ring_book_loader_test.cc)
ament_add_gtest(road_rulebook_loader_test road_rulebook_loader_test.cc)
ament_add_gtest(segment_analysis_test segment_analysis_test.cc)
ament_add_gtest(traffic_light_book_loader_test traffic_light_book_loader_test.cc)
ament_add_gtest(waypoints_test waypoints_test.cc)

add_dependencies_to_test(derive_lane_s_routes_test)
add_dependencies_to_test(find_lane_sequences_test)
add_dependencies_to_test(generate_obj_test)
add_dependencies_to_test(generate_string_test)
add_dependencies_to_test(generate_urdf_test)
add_dependencies_to_test(intersection_book_loader_test)
add_dependencies_to_test(phase_ring_book_loader_test)
add_dependencies_to_test(road_rulebook_loader_test)
add_dependencies_to_test(segment_analysis_test)
add_dependencies_to_test(traffic_light_book_loader_test)
add_dependencies_to_test(waypoints_test)
