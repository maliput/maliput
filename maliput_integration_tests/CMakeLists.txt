##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput-integration-tests LANGUAGES C CXX VERSION 3.0.0)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(drake_vendor REQUIRED)
find_package(maliput REQUIRED)
find_package(multilane REQUIRED)
find_package(dragway REQUIRED)

##############################################################################
# Tests
##############################################################################

if (BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
  add_test(NAME CLANGFORMAT COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_clang_format.sh)
endif()

ament_package()
