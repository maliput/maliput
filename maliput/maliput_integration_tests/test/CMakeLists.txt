find_package(ament_cmake_gtest REQUIRED)

macro(add_dependencies_to_test target)
    if (TARGET ${target})
      ament_target_dependencies(${target}
        "maliput"
        "multilane"
        "drake"
      )
      target_link_libraries(${target}
        multilane::multilane
        maliput::api
        maliput::maliput_test_utilities
        maliput::base
        maliput::common
        drake::drake)
    endif()
endmacro()

ament_add_gtest(intersection_book_loader_test intersection_book_loader_test.cc)
ament_add_gtest(phase_ring_book_loader_test phase_ring_book_loader_test.cc)
ament_add_gtest(road_rulebook_loader_test road_rulebook_loader_test.cc)
ament_add_gtest(traffic_light_book_loader_test traffic_light_book_loader_test.cc)

add_dependencies_to_test(intersection_book_loader_test)
add_dependencies_to_test(phase_ring_book_loader_test)
add_dependencies_to_test(road_rulebook_loader_test)
add_dependencies_to_test(traffic_light_book_loader_test)
