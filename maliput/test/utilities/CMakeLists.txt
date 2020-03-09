find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

set(TEST_PATH ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/TwoLanesRoadGeometry.mtl DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/TwoLanesRoadGeometry.obj DESTINATION ${TEST_PATH})

ament_add_gtest(mesh_simplification_test mesh_simplification_test.cc)
ament_add_gtest(mock_generate_obj_test mock_generate_obj_test.cc WORKING_DIRECTORY ${TEST_PATH})
ament_add_gtest(mock_generate_string_test mock_generate_string_test.cc)
ament_add_gtest(mock_generate_urdf_test mock_generate_urdf_test.cc)
ament_add_gtest(mock_segment_analysis_test mock_segment_analysis_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
      )

      ament_target_dependencies(${target}
          "maliput"
      )

      target_link_libraries(${target}
          api
          common
          maliput_test_utilities)
          utilities

    endif()
endmacro()

add_dependencies_to_test(mesh_simplification_test)
add_dependencies_to_test(mock_generate_obj_test)
add_dependencies_to_test(mock_generate_string_test)
add_dependencies_to_test(mock_generate_urdf_test)
add_dependencies_to_test(mock_segment_analysis_test)
