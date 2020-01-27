find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(matrix_test matrix_test.cc)
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

      ament_target_dependencies(${target}
          "drake"
      )

      target_link_libraries(${target}
          common
          drake::drake
          math
      )

    endif()
endmacro()

add_dependencies_to_test(matrix_test)
add_dependencies_to_test(saturate_test)
add_dependencies_to_test(vector_test)

ament_add_pytest_test(vector_pytest
  vector_test.py
  # Avoid pytest from importing the module stub
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
