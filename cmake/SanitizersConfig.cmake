##############################################################################
# Useful Macros
##############################################################################

# It will throw an error if more than one sanitizer is enabled.
macro(check_sanitizers_exclusivity)
  set(SANITIZER_COUNT 0)
  set(SANITIZER_LIST)
  foreach(sanitizer IN ITEMS ${ARGN})
    if(${${sanitizer}})
      math(EXPR SANITIZER_COUNT "1 + ${SANITIZER_COUNT}")
      list(APPEND SANITIZER_LIST ${sanitizer})
    endif()
  endforeach()

  if(${SANITIZER_COUNT} GREATER 1)
    message(FATAL_ERROR "Can only enable one of ${SANITIZER_LIST} at a time.")
  endif()
endmacro()

##############################################################################
# Sanitizers Configuration
##############################################################################

set(SANITIZERS off)
if ("${CMAKE_CXX_COMPILER_ID} " MATCHES "Clang ")
  option(ADDRESS_SANITIZER "Enable Clang Address Sanitizer" OFF)
  option(THREAD_SANITIZER "Enable Clang Thread Sanitizer" OFF)
  option(UNDEFINED_SANITIZER "Enable Clang Undefined Behaviour Sanitizer" OFF)

  check_sanitizers_exclusivity(ADDRESS_SANITIZER THREAD_SANITIZER UNDEFINED_SANITIZER)

  # Address Sanitizer Configuration
  if (ADDRESS_SANITIZER)
      message(STATUS "Address Sanitizer - enabled")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope")
      set(LDFLAGS "${LDFLAGS} -fsanitize=address -fno-optimize-sibling-calls -fsanitize-address-use-after-scope")
      set(SANITIZERS on)
  else()
      message(STATUS "Address Sanitizer - disabled")
  endif()

  # Thread Sanitizer Configuration
  if (THREAD_SANITIZER)
      message(STATUS "Thread Sanitizer - enabled")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fsanitize=thread")
      set(LDFLAGS "${LDFLAGS} -fsanitize=thread")
      set(SANITIZERS on)
  else()
      message(STATUS "Thread Sanitizer - disabled")
  endif()

  # Undefined Behaviour Sanitizer Configuration
  if (UNDEFINED_SANITIZER)
      message(STATUS "Undefined Behaviour Sanitizer - enabled")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
      set(LDFLAGS "${LDFLAGS} -fsanitize=undefined")
      set(SANITIZERS on)
  else()
      message(STATUS "Undefined Behaviour Sanitizer - disabled")
  endif()

else ()
    message(STATUS "Compiler Id:${CMAKE_CXX_COMPILER_ID} - Sanitizers disabled.")
endif ()
