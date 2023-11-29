ament_add_gtest(arc_length_integrator_test arc_length_integrator_test.cc)
ament_add_gtest(inverse_arc_length_integrator_test inverse_arc_length_integrator_test.cc)

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
          maliput::drake
          maliput::math
      )

    endif()
endmacro()

add_dependencies_to_test(arc_length_integrator_test)
add_dependencies_to_test(inverse_arc_length_integrator_test)
