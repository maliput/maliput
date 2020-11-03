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
# Setups alternatives for clang suite.
# Arguments:
#   Version of the clang suite package.
# Returns:
#   0 if no error was detected, 2 otherwise.
#######################################
function update_clang_suite_alternatives() {
  local version=$1
  local priority=$2

  update-alternatives \
    --install /usr/bin/clang                 clang                 /usr/bin/clang-${version}  ${priority}\
    --slave   /usr/bin/clang++               clang++               /usr/bin/clang++-${version}  \
    --slave   /usr/bin/asan_symbolize        asan_symbolize        /usr/bin/asan_symbolize-${version} \
    --slave   /usr/bin/c-index-test          c-index-test          /usr/bin/c-index-test-${version} \
    --slave   /usr/bin/clang-check           clang-check           /usr/bin/clang-check-${version} \
    --slave   /usr/bin/clang-cl              clang-cl              /usr/bin/clang-cl-${version} \
    --slave   /usr/bin/clang-cpp             clang-cpp             /usr/bin/clang-cpp-${version} \
    --slave   /usr/bin/clang-format          clang-format          /usr/bin/clang-format-${version} \
    --slave   /usr/bin/clang-format-diff     clang-format-diff     /usr/bin/clang-format-diff-${version} \
    --slave   /usr/bin/clang-import-test     clang-import-test     /usr/bin/clang-import-test-${version} \
    --slave   /usr/bin/clang-include-fixer   clang-include-fixer   /usr/bin/clang-include-fixer-${version} \
    --slave   /usr/bin/clang-offload-bundler clang-offload-bundler /usr/bin/clang-offload-bundler-${version} \
    --slave   /usr/bin/clang-query           clang-query           /usr/bin/clang-query-${version} \
    --slave   /usr/bin/clang-rename          clang-rename          /usr/bin/clang-rename-${version} \
    --slave   /usr/bin/clang-reorder-fields  clang-reorder-fields  /usr/bin/clang-reorder-fields-${version} \
    --slave   /usr/bin/clang-tidy            clang-tidy            /usr/bin/clang-tidy-${version} \
    --slave   /usr/bin/lldb                  lldb                  /usr/bin/lldb-${version} \
    --slave   /usr/bin/lldb-server           lldb-server           /usr/bin/lldb-server-${version}
}


#######################################
# Modify clang suite version and priority as required.
#######################################
CLANG_SUITE_VERSION=8
CLANG_SUITE_ALTERNATIVE_PRIORITY=10

install_clang_suite ${CLANG_SUITE_VERSION}
update_clang_suite_alternatives ${CLANG_SUITE_VERSION} ${CLANG_SUITE_ALTERNATIVE_PRIORITY}
