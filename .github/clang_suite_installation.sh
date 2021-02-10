#! /bin/bash

#######################################
# Installs clang suite packages.
# Arguments:
#   Version of the clang suite package.
# Returns:
#   0 if no error was detected, non-zero otherwise.
#######################################
function install_clang_suite() {
  local version=$1

  apt install -y \
      clang-${version} \
      lldb-${version} \
      lld-${version} \
      clang-format-${version} \
      clang-tidy-${version} \
      libc++-${version}-dev \
      libc++abi-${version}-dev
}

#######################################
# Modify clang suite version as required.
#######################################
CLANG_SUITE_VERSION=8

install_clang_suite ${CLANG_SUITE_VERSION}
