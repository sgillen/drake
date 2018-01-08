# Stub functionality for macros to be consumed by downstream packages.

def external_data_stub_test():
    # Import test from this package in downstream packages.
    alias(
        name = "external_data_stub_test",
        actual = "@bazel_external_data_pkg//src/bazel_external_data:stub_test",
        testonly = 1,
    )
