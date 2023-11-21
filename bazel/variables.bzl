###############################################################################
# Build Variables
###############################################################################

# TODO(daniel.stonier): revise these, get in sync with cmake/DefaultCFlags.cmake.
# Be aware though that many do not work across platforms, e.g. there is no
# cross-platform mechanism for setting std=c++17 in bazel

COPTS = [
    "-std=c++17",
    "-Wno-builtin-macro-redefined",
    "-Wno-missing-field-initializers",
    "-Wno-unused-const-variable",
    "-O2",

    # Others from cmake/DefaultCFlags.cmake
    # "-fdata-sections",
    # "-fdiagnostics-color=always"
    # "-ffunction-sections"
    # "-fopenmp"
    # "-fPIC"
    # "-fstack-protector"
    # "-fno-omit-frame-pointer"
    # "-no-canonical-prefixes"
    # "-Wall"
    # "-Wregister"
    # "-Wstrict-overflow"

    # Some flags that were used in the TRI build
    # "-Wno-unused-parameter",
    # "-Wno-missing-braces",
    # "-Wno-pessimizing-move",
    # "-Wno-self-assign",
    # "-Wno-deprecated-declarations",
    # "-Wno-unused-private-field",
    # "-Wno-maybe-uninitialized",
    # "-Wno-deprecated-register",
]
