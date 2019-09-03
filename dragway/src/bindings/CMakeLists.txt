##############################################################################
# Sources
##############################################################################

pybind11_add_module(dragway_py MODULE dragway_py.cc)
if(PYTHON_IS_DEBUG)
  target_compile_definitions(dragway_py PRIVATE Py_DEBUG)
endif()

set_target_properties(dragway_py PROPERTIES OUTPUT_NAME "dragway")

target_include_directories(dragway_py
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(dragway_py
  PRIVATE dragway maliput::api
)

##############################################################################
# Install
##############################################################################

install(TARGETS dragway_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
