##############################################################################
# Sources
##############################################################################

# api module
pybind11_add_module(api_py api_py.cc)

set_target_properties(api_py PROPERTIES OUTPUT_NAME "api")

ament_target_dependencies(api_py
  "maliput"
)

target_include_directories(api_py
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(api_py
  PRIVATE
    maliput::api
    pybind11::module
)

# math module
pybind11_add_module(math_py math_py.cc)

set_target_properties(math_py PROPERTIES OUTPUT_NAME "math")

target_include_directories(math_py
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

ament_target_dependencies(math_py
  "maliput"
)

target_link_libraries(math_py
  PRIVATE
    maliput::math
    pybind11::module
)

##############################################################################
# Install
##############################################################################

message(STATUS "********* ${PYTHON_INSTALL_DIR}")
install(TARGETS api_py
  DESTINATION "${PYTHON_INSTALL_DIR}/maliput"
)

install(TARGETS math_py
  DESTINATION "${PYTHON_INSTALL_DIR}/maliput"
)
