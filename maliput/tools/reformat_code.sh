#!/bin/bash

# This is a simple script to run the ament_clang_format against all of the code
# in the repository.  ament_clang_format will reformat all of the code according
# to the format described in the top-level .clang-format file in this
# repository.  It is recommended to run this before opening any pull request.

SCRIPT_PATH=$(realpath ${BASH_SOURCE[0]})
SCRIPT_DIR=$(dirname $SCRIPT_PATH)
REPO_DIR=$SCRIPT_DIR/..

pushd $REPO_DIR
./tools/run_clang_format.sh --reformat
popd
