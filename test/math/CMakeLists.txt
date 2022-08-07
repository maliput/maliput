find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

ament_add_gtest(matrix_test matrix_test.cc)
ament_add_gtest(quaternion_test quaternion_test.cc)
ament_add_gtest(roll_pitch_yaw_test roll_pitch_yaw_test.cc)
ament_add_gtest(saturate_test saturate_test.cc)
ament_add_gtest(vector_test vector_test.cc)
ament_add_gtest(kd_tree_test kd_tree_test.cc)
ament_add_gtest(overlapping_type_test overlapping_type_test.cc)
ament_add_gtest(bounding_box_test bounding_box_test.cc)
ament_add_gtest(axis_aligned_box_test axis_aligned_box_test.cc)
ament_add_gmock(bounding_region_test bounding_region_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::common
          maliput::math
          maliput::test_utilities
      )

    endif()
endmacro()

add_dependencies_to_test(matrix_test)
add_dependencies_to_test(quaternion_test)
add_dependencies_to_test(roll_pitch_yaw_test)
add_dependencies_to_test(saturate_test)
add_dependencies_to_test(vector_test)
add_dependencies_to_test(kd_tree_test)
add_dependencies_to_test(overlapping_type_test)
add_dependencies_to_test(bounding_box_test)
add_dependencies_to_test(axis_aligned_box_test)
add_dependencies_to_test(bounding_region_test)
