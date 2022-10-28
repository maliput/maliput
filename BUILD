cc_library(
    name = "api",
    srcs = glob(["src/api/*.cc"]),
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
    deps = ["@yaml-cpp//:yaml-cpp", "@//:common", '@//:math', '@//:api']
)

cc_library(
    name = "common",
    srcs = glob(["src/common/*.cc"]),
    hdrs = glob(["include/maliput/common/*.h"]),
    strip_include_prefix = "include",
    deps = ["@fmt//:fmt"],

)

cc_library(
    name = "geometry_base",
    srcs = glob(["src/geometry_base/*.cc"]),
    hdrs = glob([
        "include/maliput/geometry_base/*.h",
    ]),
    strip_include_prefix = "include",
    deps = ['@//:common', '@//:math', '@//:api']
)

cc_library(
    name = "math",
    srcs = glob(["src/math/*.cc"]),
    hdrs = glob([
        "include/maliput/math/*.h",
    ]),
    strip_include_prefix = "include",
    deps = ['@//:common']
)

cc_library(
    name = "plugin",
    srcs = glob(["src/plugin/*.cc"]),
    hdrs = glob([
        "include/maliput/plugin/*.h",
        "include/maliput/api/*.h",
        "include/maliput/math/*.h",
    ]),
    strip_include_prefix = "include",
    deps = ['@//:api', '@//:common', '@//:math', '@//:utility']
)

cc_library(
    name = "routing",
    srcs = glob(["src/routing/*.cc"]),
    hdrs = glob([
        "include/maliput/routing/*.h",
    ]),
    strip_include_prefix = "include",
    deps = ['@//:math', '@//:common', '@//:api']
)

cc_library(
    name = "test_utilities",
    srcs = glob(["src/test_utilities/*.cc"]),
    hdrs = glob([
        "include/maliput/test_utilities/*.h",
    ]),
    strip_include_prefix = "include",
    deps = [
        "@com_google_googletest//:gtest",
        '@//:geometry_base',
        "@//:api", "@//:common", "@//:math",
    ]
)

cc_library(
    name = "utility",
    srcs = glob(["src/utility/*.cc"]),
    hdrs = glob(["include/maliput/utility/*.h"]),
    strip_include_prefix = "include",
    deps = ['@//:api', '@//:common', '@//:math']
)

#### test #####
cc_test(
    name = "maliput_tes",
    size = "small",
    srcs = glob([
        "test/api/*.cc",
        "test/base/*.cc",
        "test/common/*.cc",
        "test/geometry_base/*.cc",
        "test/math/*.cc",
        "test/plugin/*.cc",
        "test/utility/*.cc",
    ]),
    deps = [
        "@com_google_googletest//:gtest_main",
        "@//:test_utilities",
        "@//:common",
        "@//:math",
        "@//:api",
        "@//:base",
        "@//:utility",
        "@//:plugin",
        "@//:routing"
    ],
)