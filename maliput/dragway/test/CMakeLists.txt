find_package(ament_cmake_gtest REQUIRED)

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
          maliput::test_utilities)

    endif()
endmacro()

add_dependencies_to_test(dragway_test)
