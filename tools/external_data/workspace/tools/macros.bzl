# -*- python -*-

# Stub functionality for macros to be consumed by downstream packages.

def external_data_stub_test():
    # Redefine existing stub test.
    file = "@bazel_external_data_pkg//src/bazel_external_data:stub_test.py"
    native.py_test(
        name = "stub_test",
        srcs = [file],
        main = file,
    )
