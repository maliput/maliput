find_package(ament_cmake_pytest REQUIRED)

ament_add_pytest_test(yaml_to_obj_test yaml_to_obj_test.py WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/bin TIMEOUT 360)

if (TARGET yaml_to_obj_test)
  add_dependencies(yaml_to_obj_test yaml_to_obj)
endif()
