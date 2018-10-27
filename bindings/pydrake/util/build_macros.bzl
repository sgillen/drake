load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/skylark:drake_cc.bzl", "drake_cc_library")

_HEADER_TEMPLATE = """
#pragma once

// N.B. This is an auto-generated header for backwards compatibility.
// This header will be deprecated on 2018/12/01.
#include <drake/{package}/{header}>
""".lstrip()

def util_cc_header_alias(name):
    package = "bindings/pydrake/common"
    header = name + ".h"
    args = dict(
        package = package,
        header = header,
    )

    # TODO(eric.cousineau): Add deprecation message on 2018/11/01.
    generate_file(
        name = header,
        content = _HEADER_TEMPLATE.format(**args),
    )
    drake_cc_library(
        name = name,
        hdrs = [header],
        deps = ["//" + package + ":" + name],
    )

def util_target_aliases(names):
    for name in names:
        native.alias(
            name = name,
            actual = "//bindings/pydrake/common:" + name,
        )
