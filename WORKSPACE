workspace(name="maliput")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "fmt",
    urls = ["https://github.com/fmtlib/fmt/releases/download/9.1.0/fmt-9.1.0.zip"],
    strip_prefix = "fmt-9.1.0",
    build_file = "@//external:fmt.BUILD"
)

http_archive(
    name = "yaml-cpp",
    urls = ["https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.7.0.zip"],
    strip_prefix = "yaml-cpp-yaml-cpp-0.7.0"
)

http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip"],
    strip_prefix = "googletest-release-1.12.1",
)