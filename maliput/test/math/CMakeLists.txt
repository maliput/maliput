find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(matrix_test matrix_test.cc)
ament_add_gtest(quaternion_test quaternion_test.cc)
ament_add_gtest(roll_pitch_yaw_test roll_pitch_yaw_test.cc)
ament_add_gtest(saturate_test saturate_test.cc)
ament_add_gtest(vector_test vector_test.cc)

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
