#!/bin/bash

# BSD 3-Clause License
#
# Copyright (c) 2022, Woven Planet. All rights reserved.
# Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
ament_clang_format --config=./.clang-format --reformat || CLANGFORMATFAILED=1
popd

if [ "$CLANGFORMATFAILED" -ne "0" ]; then
  echo $'\n*** ament_clang_format failed ***'
  exit 1
fi
