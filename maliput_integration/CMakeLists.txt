##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput-integration LANGUAGES C CXX VERSION 3.0.0)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

set(BUILD_SHARED_LIBS true)

include(${PROJECT_SOURCE_DIR}/../cmake/DefaultCFlags.cmake)

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# Tests
##############################################################################

# We always enable tests (that might change in the future)
if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  enable_testing()
  add_subdirectory(test)
  # TODO: when adding code, this should be re-enabled.
  # ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format)
endif()

##############################################################################
# Docs
##############################################################################

if(NOT DEFINED BUILD_DOCS)
  set(BUILD_DOCS 1)
endif()

if(BUILD_DOCS)
  find_package(ament_cmake_doxygen REQUIRED)
  ament_doxygen_generate(doxygen_maliput_integration
    CONFIG_OVERLAY doc/Doxyfile.overlay.in
  )
endif()

##############################################################################
# Export
##############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_package()
