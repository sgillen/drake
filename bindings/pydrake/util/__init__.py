# Backwards-compatibility aliases.
drake_py_library(
    name = "third_party",
    visibility = ["//visibility:public"],
    deps = PY_LIBRARIES,
)
