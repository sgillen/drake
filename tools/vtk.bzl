# -*- python -*-

"""
Downloads and unpacks a VTK archive and makes its headers and
precompiled shared libraries available to be used as a C/C++
dependency.
"""

def _exec_or_fail(repository_ctx, arguments):
    result = repository_ctx.execute(arguments)
    if result.return_code != 0:
        print(result.return_code, result.stdout, result.stderr)
        fail("Could not execute '" + "' '".join(arguments) + "'")
    return result

def _lsb_release(repository_ctx, option):
    lsb_release = repository_ctx.which("lsb_release")
    stdout = _exec_or_fail(repository_ctx, [lsb_release, option]).stdout
    key, value = stdout.split(':')
    return value.strip()

def _ubuntu_impl(repository_ctx, release):
    name = repository_ctx.name
    dpkg = repository_ctx.which("dpkg")

    # Figure out which archive to use.  The sha256 should be the last entry in
    # the platform-specific list of archives, but if the user omitted the
    # checksum, then we set a bogus one instead.  This is a convenience to
    # allow Bazel to report the correct value when switching to new archive.
    platform_urls = repository_ctx.attr.platform_urls
    platform_key = "Ubuntu/" + release
    urls = [x for x in platform_urls.get(platform_key, [])]
    sha256 = "0" * 64
    if urls and urls[-1].startswith("sha256:"):
        sha256 = urls.pop()[len("sha256:"):]
    if not urls:
        fail("Missing WORKSPACE {} platform_urls for {}".format(
            name, platform_key))

    # Download the archive.
    repository_ctx.download_and_extract(urls, ".", sha256=sha256)

    # TODO(jwnimmer-tri) Instead of a single library with everything, we should
    # be able to have a small python program parse the *.cmake files and emit a
    # whole bunch of BUILD targets.  Alternatively, walking the *.so files to
    # extract their names and depdendencies might also work.
    rpath = repository_ctx.path("lib")
    build_content = """
package(default_visibility = ["//visibility:public"])
ALL_FILES = glob(["**/*"], exclude=["BUILD", "WORKSPACE"])
exports_files(ALL_FILES)
filegroup(
    name = "all_files",
    srcs = ALL_FILES,
)
cc_library(
    name = "vtk_libraries",
    hdrs = glob(["include/**/*"]),
    includes = ["include/vtk-7.1"],
    srcs = glob(["lib/*.so"]),
    linkopts=["-Wl,-rpath {}"],
)
    """.format(rpath)
    repository_ctx.file("BUILD", content=build_content)

def _linux_impl(repository_ctx):
    distributor_id = _lsb_release(repository_ctx, "--id")
    release = _lsb_release(repository_ctx, "--release")
    if distributor_id == "Ubuntu":
        return _ubuntu_impl(repository_ctx, release)
    fail("Linux '{} {}' is NOT supported".format(distributor_id, release))

def _impl(repository_ctx):
    os_name = repository_ctx.os.name
    if os_name == "linux":
        return _linux_impl(repository_ctx)
    fail("Operating system '{}' is NOT supported".format(os_name))

vtk_repository = repository_rule(
    attrs = {
        "platform_urls": attr.string_list_dict(
            mandatory = True,
            allow_empty = False,
        ),
    },
    implementation = _impl,
)
