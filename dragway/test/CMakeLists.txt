find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(dragway_test dragway_test.cc)

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
          dragway
          maliput::maliput_test_utilities)

    endif()
endmacro()

add_dependencies_to_test(dragway_test)

ament_add_pytest_test(dragway_pytest
  dragway_test.py
  # Avoid pytest from importing the module stub
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
