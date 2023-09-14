workspace(name = "maliput")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "fmt",
    build_file = "@//external:fmt.BUILD",
    strip_prefix = "fmt-6.1.2",
    urls = ["https://github.com/fmtlib/fmt/releases/download/6.1.2/fmt-6.1.2.zip"],
)

http_archive(
    name = "yaml-cpp",
    build_file = "@//external:yaml-cpp.BUILD",
    strip_prefix = "yaml-cpp-yaml-cpp-0.6.2",
    urls = ["https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.6.2.zip"],
)

http_archive(
    name = "com_google_googletest",
    strip_prefix = "googletest-release-1.12.1",
    urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip"],
)
