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
    size = "enormous",
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
    size = "enormous",
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
    size = "enormous",
    timeout = "long",
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
    size = "enormous",
    timeout = "long",
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
    size = "enormous",
    timeout = "long",
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
    size = "enormous",
    timeout = "long",
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
    size = "enormous",
    timeout = "long",
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
    name = "utlity_test",
    size = "enormous",
    timeout = "long",
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
