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

declare -i CLANGFORMATFAILED=0

pushd $REPO_DIR
ament_clang_format --config=./../.clang-format $1 || CLANGFORMATFAILED=1
popd

if [ "$CLANGFORMATFAILED" -ne "0" ]; then
  echo $'\n*** ament_clang_format failed ***'
  exit 1
fi
