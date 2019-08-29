#!/bin/bash

# This script runs clang-format and reformats any C++ code in 
# the package. It expects a .clang-format to exist within the
# root directory of the project.  To stop a directory from being
# reformatted, add a blank `AMENT_IGNORE` file to the root of
# the directory to exclude.

SCRIPT_PATH=$(realpath ${BASH_SOURCE[0]})
SCRIPT_DIR=$(dirname $SCRIPT_PATH)
REPO_DIR=$SCRIPT_DIR/..

export PATH=$PATH:/home/$USER/.local/bin

declare -i CLANGFORMATFAILED=0

pushd $REPO_DIR
ament_clang_format --config=./../.clang-format --reformat || CLANGFORMATFAILED=1
popd

if [ "$CLANGFORMATFAILED" -ne "0" ]; then
  echo $'\n*** ament_clang_format failed ***'
  exit 1
fi
