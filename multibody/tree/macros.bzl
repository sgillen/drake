load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_library",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

TEMPLATE = """
#pragma once

#include "drake/multibody/tree/internal/{header}"

#warning "This header is deprecated, and will be removed around 2019/03/01."
""".lstrip()

def drake_cc_forwarding_hdrs_internal(
        name,
        hdrs = None,
        visibility = None):
    hdrs or fail("`hdrs` must be supplied")
    for header in hdrs:
        generate_file(
            name = header,
            content = TEMPLATE.format(header = header),
            visibility = visibility,
        )
    drake_cc_library(
        name = name,
        hdrs = hdrs,
        visibility = visibility,
    )
