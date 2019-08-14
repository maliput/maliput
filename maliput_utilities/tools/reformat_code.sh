#!/bin/bash

# This is a simple script to run the ament_clang_format against all of the code
# in the repository.  ament_clang_format will reformat all of the code according
# to the format described in the top-level .clang-format file in this
# repository.  It is recommended to run this before opening any pull request.

./run_clang_format.sh --reformat
# ./run_clang_tidy.sh --fix-errors
