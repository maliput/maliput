# BSD 3-Clause License

# Copyright (c) 2022, Woven Planet.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.

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

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "api",
    srcs = glob([
        "src/api/*.cc",
        "src/api/rules/*.cc",
    ]),
    hdrs = glob([
        "include/maliput/api/*.h",
        "include/maliput/api/rules/*.h",
        "include/maliput/base/*.h",
        "include/maliput/common/*.h",
        "include/maliput/math/*.h",
    ]),
    strip_include_prefix = "include",
)

cc_library(
    name = "base",
    srcs = glob(["src/base/*.cc"]),
    hdrs = glob([
        "include/maliput/base/*.h",
    ]),
    strip_include_prefix = "include",
    deps = [
        "@//:api",
        "@//:common",
        "@//:math",
        "@yaml-cpp//:yaml-cpp",
    ],
)

cc_library(
    name = "common",
    srcs = glob(["src/common/*.cc"]),
    hdrs = glob(["include/maliput/common/*.h"]),
    strip_include_prefix = "include",
    deps = ["@fmt"],
)

cc_library(
    name = "geometry_base",
    srcs = glob(["src/geometry_base/*.cc"]),
    hdrs = glob([
        "include/maliput/geometry_base/*.h",
    ]),
    strip_include_prefix = "include",
    deps = [
        "@//:api",
        "@//:common",
        "@//:math",
    ],
)

cc_library(
    name = "math",
    srcs = glob(["src/math/*.cc"]),
    hdrs = glob([
        "include/maliput/math/*.h",
    ]),
    strip_include_prefix = "include",
    deps = ["@//:common"],
)

cc_library(
    name = "plugin",
    srcs = glob(["src/plugin/*.cc"]),
    hdrs = glob([
        "include/maliput/plugin/*.h",
        "include/maliput/api/*.h",
        "include/maliput/math/*.h",
    ]),
    linkopts = ["-ldl"],
    strip_include_prefix = "include",
    deps = [
        "@//:api",
        "@//:common",
        "@//:math",
        "@//:utility",
    ],
)

cc_library(
    name = "routing",
    srcs = glob(["src/routing/*.cc"]),
    hdrs = glob([
        "include/maliput/routing/*.h",
    ]),
    strip_include_prefix = "include",
    deps = [
        "@//:api",
        "@//:common",
        "@//:math",
    ],
)

cc_library(
    name = "test_utilities",
    srcs = glob(["src/test_utilities/*.cc"]),
    hdrs = glob([
        "include/maliput/test_utilities/*.h",
    ]),
    strip_include_prefix = "include",
    deps = [
        "@//:api",
        "@//:common",
        "@//:geometry_base",
        "@//:math",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "utility",
    srcs = glob(["src/utility/*.cc"]),
    hdrs = glob(["include/maliput/utility/*.h"]),
    linkopts = ["-lstdc++fs"],
    strip_include_prefix = "include",
    deps = [
        "@//:api",
        "@//:common",
        "@//:math",
    ],
)

#### test #####
cc_test(
    name = "api_test",
    size = "large",
    timeout = "long",
    srcs = glob(["test/api/*.cc"]),
    deps = [
        "@//:api",
        "@//:common",
        "@//:math",
        "@//:test_utilities",
        "@//:utility",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "base_test",
    size = "large",
    timeout = "long",
    srcs = glob([
        "test/base/*.cc",
    ]),
    deps = [
        "@//:api",
        "@//:base",
        "@//:test_utilities",
        "@com_google_googletest//:gtest_main",
        "@fmt",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    timeout = "short",
    srcs = glob([
        "test/common/*.cc",
    ]),
    deps = [
        "@//:common",
        "@//:test_utilities",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "geometry_base_test",
    size = "small",
    timeout = "short",
    srcs = glob([
        "test/geometry_base/*.cc",
    ]),
    deps = [
        "@//:api",
        "@//:test_utilities",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "math_test",
    size = "large",
    timeout = "moderate",
    srcs = glob([
        "test/math/*.cc",
    ]),
    deps = [
        "@//:common",
        "@//:math",
        "@//:test_utilities",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "plugin_test",
    size = "large",
    timeout = "moderate",
    srcs = glob([
        "test/plugin/*.cc",
    ]),
    local_defines = [
        "TEST_MALIPUT_PLUGIN_LIBDIR=\\\"/tmp/maliput/test/plugins/\\\"",
        "OTHER_TEST_MALIPUT_PLUGIN_LIBDIR=\\\"/tmp/maliput/test/other_plugins/\\\"",
    ],
    deps = [
        "@//:api",
        "@//:plugin",
        "@//:test_utilities",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_utilities_test",
    size = "small",
    timeout = "short",
    srcs = glob([
        "test/test_utilities/*.cc",
    ]),
    deps = [
        "@//:api",
        "@//:common",
        "@//:test_utilities",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "utility_test",
    size = "large",
    timeout = "moderate",
    srcs = glob([
        "test/utility/*.cc",
    ]),
    deps = [
        "@//:api",
        "@//:common",
        "@//:test_utilities",
        "@//:utility",
        "@com_google_googletest//:gtest_main",
    ],
)
