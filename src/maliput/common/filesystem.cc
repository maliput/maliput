// clang-format off

// Code in this file is inspired by:
// https://github.com/psiorx/spruce-pod/blob/master/src/spruce.cc
//
// Code license follows:
// Copyright (c) 2014, Joseph Davis (@josephdavisco)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the project nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "maliput/common/filesystem.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>

extern "C" {
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}

namespace maliput {
namespace common {

Path::Path(std::string path_) : path(path_)
{}

void Path::set_path(const std::string& path_) noexcept {
    path = path_;
    normalize();
}

void Path::set_as_temp()
{
    char* cwd = std::getenv("TMPDIR");
    if (!cwd) cwd = std::getenv("TEMPDIR");
    if (!cwd) cwd = std::getenv("TMP");
    if (!cwd) cwd = std::getenv("TEMP");
    if (!cwd) {
    set_path("/tmp");
    } else {
    set_path(cwd);
    }
}

const std::string &Path::get_path() const noexcept {
    return path;
}

bool Path::is_file() const noexcept{
    if (!exists()){
        return false;
    }
    struct stat attributes;
    stat(path.c_str(), &attributes);

    return (attributes.st_mode & S_IFMT) == S_IFREG;
}

bool Path::is_directory() const noexcept {
    if (!exists()){
        return false;
    }
    struct stat attributes;
    stat(path.c_str(), &attributes);
    return (attributes.st_mode & S_IFMT) == S_IFDIR;
}

bool Path::is_absolute() const noexcept {
    return exists() && !path.empty() && path[0] == '/';
}

bool Path::exists() const noexcept {
    struct stat attributes;
    return stat(path.c_str(), &attributes) == 0;
}

void Path::append(const std::string& path_) {
    if (path_.length() > 0 && *path_.begin() == '/')
    {
        path = path_;
    }
    else
    {
        path.append('/' + path_);
    }
    normalize();
}

void Path::normalize() {
    if (path.back() == '/' && path.length() > 1) {
        path = path.substr(0, path.length() - 1);
    }
}

bool Filesystem::create_directory(const Path& path) {
    return mkdir(path.get_path().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0;
}

bool Filesystem::remove_directory(const Path& path) {
    return path.is_directory() && rmdir(path.get_path().c_str()) == 0;
}

bool Filesystem::remove_file(const Path& path) {
    return path.is_file() && (std::remove((path.get_path()).c_str()) == 0);
}

Path Filesystem::get_cwd() {
    char* cwd = getcwd(NULL, 0);
    Path path(cwd);
    free(cwd);
    return path;
}

bool Filesystem::read_as_string(const Path &path, std::string &read_to) {
    std::ifstream is(path.get_path());
    std::stringstream ss;
    if (is.is_open()) {
        while (true) {
            char c = is.get();
            if (is.eof()) { break; }
            ss << c;
        }
        read_to = ss.str();
        return true;
    }
    return false;
}

bool Filesystem::create_directory_recursive(const Path &path) {
    size_t pos = 1;
    std::string temp(path.get_path());

    if (temp == "") {
        return false;
    }
    while ((pos = temp.find('/', pos + 1)) != std::string::npos) {

        if ((Path(temp.substr(0, pos))).exists()) {
            continue;
        }
        // if making a directory on the path fails we cannot continue
        if (!create_directory(temp.substr(0, pos))) {
            return false;
        }
    }
    return create_directory(Path(temp));
}

std::string Filesystem::get_env_path(const std::string &env_var) {
    char *env_path = std::getenv(env_var.c_str());
    if (env_path)
    {
        return std::string(env_path);
    }
    return std::string();
}

} // namespace common
} // namespace maliput
// clang-format on
