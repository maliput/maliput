##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput-integration-tests LANGUAGES C CXX VERSION 3.0.0)
option(BUILD_TESTS "Build test" OFF)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(drake REQUIRED)
find_package(maliput REQUIRED)
find_package(multilane REQUIRED)

##############################################################################
# Tests
##############################################################################

if (BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
