find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(maliput_types_compare_test maliput_types_compare_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::api
          maliput::common
          maliput::test_utilities
      )

    endif()
endmacro()

add_dependencies_to_test(maliput_types_compare_test)
