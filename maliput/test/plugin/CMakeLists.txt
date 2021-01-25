ament_add_gtest(maliput_plugin_test maliput_plugin_test.cc)
ament_add_gtest(maliput_plugin_manager_test maliput_plugin_manager_test.cc)

# Variables holding the location to where
# some testing library are installed.
# Additionaly, these variables are passed as macros to the test
# files using the target_compile_definitions feature.
set(TEST_PLUGIN_INSTALL_DIR
    "/tmp/maliput/test/plugins/")
set(OTHER_TEST_PLUGIN_INSTALL_DIR
    "/tmp/maliput/test/other_plugins/")

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

      target_compile_definitions(${target}
        PRIVATE
          TEST_MALIPUT_PLUGIN_LIBDIR="${TEST_PLUGIN_INSTALL_DIR}"
          OTHER_TEST_MALIPUT_PLUGIN_LIBDIR="${OTHER_TEST_PLUGIN_INSTALL_DIR}"
      )
    endif()
endmacro()

add_dependencies_to_test(maliput_plugin_test)
add_dependencies_to_test(maliput_plugin_manager_test)

add_subdirectory(tools)
