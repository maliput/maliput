ament_add_gtest(maliput_plugin_test maliput_plugin_test.cc)
ament_add_gtest(maliput_plugin_manager_test maliput_plugin_manager_test.cc)

# Variables holding the location where
# the testing libraries are installed.
# Additionaly, these variables are passed as macros to the test
# files using the target_compile_definitions function.
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

      # To avoid a false positive when running ubsan the symbols must be exported
      # See https://stackoverflow.com/questions/57361776/use-ubsan-with-dynamically-loaded-shared-libraries
      set_target_properties(${target}
        PROPERTIES
          ENABLE_EXPORTS ON
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
