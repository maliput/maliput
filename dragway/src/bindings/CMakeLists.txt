##############################################################################
# Sources
##############################################################################

pybind11_add_module(dragway_py dragway_py.cc)

set_target_properties(dragway_py PROPERTIES OUTPUT_NAME "dragway")

target_include_directories(dragway_py
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(dragway_py
  PRIVATE dragway maliput::api pybind11::module
)

##############################################################################
# Install
##############################################################################

install(TARGETS dragway_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
