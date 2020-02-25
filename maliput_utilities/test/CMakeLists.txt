find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

set(TEST_PATH ${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(generate_obj_test generate_obj_test.cc WORKING_DIRECTORY ${TEST_PATH})
ament_add_gtest(generate_string_test generate_string_test.cc WORKING_DIRECTORY ${TEST_PATH})
ament_add_gtest(generate_urdf_test generate_urdf_test.cc WORKING_DIRECTORY ${TEST_PATH})
ament_add_gtest(mesh_simplification_test mesh_simplification_test.cc WORKING_DIRECTORY ${TEST_PATH})
ament_add_gtest(mock_segment_analysis_test mock_segment_analysis_test.cc)
ament_add_pytest_test(yaml_to_obj_test yaml_to_obj_test.py WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/bin TIMEOUT 360)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
      )

      ament_target_dependencies(${target}
          "multilane"
          "maliput"
          "drake"
      )

      target_link_libraries(${target}
          maliput-utilities
          multilane::multilane
          maliput::api
          maliput::common
          maliput::maliput_test_utilities
          drake::drake)

    endif()
endmacro()

add_dependencies_to_test(generate_obj_test)
add_dependencies_to_test(generate_string_test)
add_dependencies_to_test(generate_urdf_test)
add_dependencies_to_test(mesh_simplification_test)
add_dependencies_to_test(mock_segment_analysis_test)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/MtlContent.mtl DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/ChangeOrigin.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/DefaultObjContent.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/HighlightedSegments.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/NoBranchPointsObjContent.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/StackedBranchPointsObjContent.obj DESTINATION ${TEST_PATH})
