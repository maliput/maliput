##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(multilane LANGUAGES C CXX VERSION 3.0.0)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(drake_vendor REQUIRED)
find_package(maliput REQUIRED)
find_package(yaml-cpp REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

set(BUILD_SHARED_LIBS true)

add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(resources)

##############################################################################
# Tests
##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  enable_testing()
  add_subdirectory(test)
  ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format)
endif()

##############################################################################
# Docs
##############################################################################

if(BUILD_DOCS)
  find_package(ament_cmake_doxygen REQUIRED)
  ament_doxygen_generate(doxygen_multilane
    CONFIG_OVERLAY doc/Doxyfile.overlay.in
    DEPENDENCIES maliput
  )
endif()

##############################################################################
# Export
##############################################################################

ament_environment_hooks(setup.sh.in)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(drake)
ament_export_dependencies(maliput)
ament_export_dependencies(yaml-cpp)
ament_export_interfaces(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_package()
