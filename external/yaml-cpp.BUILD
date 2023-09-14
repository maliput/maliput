cc_library(
    name = "yaml-cpp",
    hdrs = glob([
        "include/yaml-cpp/*.h",
        "include/yaml-cpp/node/*.h",
        "include/yaml-cpp/node/detail/*.h",
        "include/yaml-cpp/contrib/*.h",
    ]),
    srcs = glob([
        "src/*.cpp",
        "src/*.h",
        "src/contrib/*.cpp",
        "src/contrib/*.h",
    ]),
    visibility = ["//visibility:public"],
    strip_include_prefix = "include",
)
