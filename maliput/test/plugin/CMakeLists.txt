ament_add_gtest(maliput_plugin_test maliput_plugin_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::plugin
      )

    endif()
endmacro()

add_dependencies_to_test(maliput_plugin_test)

add_subdirectory(tools)
