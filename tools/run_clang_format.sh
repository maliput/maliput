#!/bin/bash
SCRIPT_PATH=$(realpath ${BASH_SOURCE[0]})
SCRIPT_DIR=$(dirname $SCRIPT_PATH)
REPO_DIR=$SCRIPT_DIR/..

export PATH=$PATH:/home/$USER/.local/bin

check_program_installed() {
  if ! [ -x "$(command -v $1)" ]; then
    printf "Error: %s was not found in your system.\n" "$1"
    printf "Please install it by running:\n\n"
    printf "sudo apt install %s\n\n" "$1"
    exit 1
  fi
}

check_program_installed ament_clang_format

DIR_NAME='./'$1
REGEX='/.*/.*\.\(c\|cc\|cpp\|cxx\|h\|hh\|hpp\|hxx\)$'

declare -i CLANGFORMATFAILED=0

# Exclude Dirs:
#  - build/style helper scripts in ./tools
#  - test helper scripts in test/utils
#  - entry points in python/examples
if [ "$CLANGFORMATFAILED" -eq "0" ]; then
  pushd $REPO_DIR    
  # Run ament_clang_format
  find -regex $DIR_NAME$REGEX -printf '%h\n' | sort | uniq | xargs ament_clang_format --config=.clang-format || CLANGFORMATFAILED=1
  popd
else
  echo $'\n*** ament_clang_format failed, not doing style formatting ***'
  exit 1
fi

if [ "$CLANGFORMATFAILED" -ne "0" ]; then
  echo $'\n*** ament_clang_format failed ***'
  exit 1
fi

