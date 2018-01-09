# -*- python -*-

# Stub functionality for macros to be consumed by downstream packages.

def external_data_stub_test():
    # Define stub test using upstream package's file.
    file = "@bazel_external_data_pkg//src/bazel_external_data:stub_test.py"
    native.py_test(
        name = "stub_test",
        srcs = [file],
        main = file,
    )
