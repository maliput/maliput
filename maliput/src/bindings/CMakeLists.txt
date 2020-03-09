##############################################################################
# Sources
##############################################################################

# api module
pybind11_add_module(api_py api_py.cc)

set_target_properties(api_py PROPERTIES OUTPUT_NAME "api")

target_include_directories(api_py
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(api_py
  PRIVATE api pybind11::module
)

# math module
pybind11_add_module(math_py math_py.cc)

set_target_properties(math_py PROPERTIES OUTPUT_NAME "math")

target_include_directories(math_py
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(math_py
  PRIVATE math pybind11::module
)

##############################################################################
# Install
##############################################################################

install(TARGETS api_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

install(TARGETS math_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
