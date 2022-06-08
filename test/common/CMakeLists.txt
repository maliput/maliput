ament_add_gtest(logger_test logger_test.cc)
ament_add_gtest(passkey_test passkey_test.cc)
ament_add_gtest(maliput_deprecated_test maliput_deprecated_test.cc)
ament_add_gtest(maliput_hash_test maliput_hash_test.cc)
ament_add_gtest(maliput_never_destroyed_test maliput_never_destroyed_test.cc)
ament_add_gtest(maliput_throw_test maliput_throw_test.cc)

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
      )

    endif()
endmacro()

add_dependencies_to_test(logger_test)
add_dependencies_to_test(passkey_test)
add_dependencies_to_test(maliput_deprecated_test)
add_dependencies_to_test(maliput_hash_test)
add_dependencies_to_test(maliput_never_destroyed_test)
add_dependencies_to_test(maliput_throw_test)

# TODO(#335): When a sanitizer is enabled, tests where assert are checked are disabled.
if (NOT ${SANITIZERS})
  ament_add_gtest(maliput_abort_test maliput_abort_test.cc)
  add_dependencies_to_test(maliput_abort_test)
endif()
