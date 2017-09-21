load("@kythe//tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

SUBMODULES = [
    "common",
    "sample_consensus",
    "search",
    "kdtree",
    "octree",
    "io",
    "filters",
    "segmentation",
    "visualization",
    ]

PKG_CFG_VERSION = "1.8"

def _impl(ctx):
    deps = []
    content = ""

    for sub in SUBMODULES:
        sub_name = "pcl_{}".format(sub)
        sub_modname = "{}-{}".format(sub_name, PKG_CFG_VERSION)
        pkg_mods.append(sub_modname)
        deps.append("@{}".format(sub_name))
        # Submodule.
        content += """
pkg_config_package(
    name = "{}",
    modname = "{}",
)""".format(sub_name, sub_modname)
        # Submodule.

    # Generate main target.
    content += """
cc_library(
    name = "{}",
    deps = {},
)""".format(ctx.name, deps)

    ctx.file(
        "BUILD",
        content = content,
        executable = False,
    )

pcl_repository = repository_rule(implementation = _impl)
