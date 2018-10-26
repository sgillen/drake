from __future__ import print_function

import argparse
from os import listdir, symlink, mkdir
from os.path import abspath, dirname, isabs, isdir, isfile, join
from shutil import rmtree
from subprocess import check_call
import sys

from refresh_doc_modules import refresh_doc_modules

_SPHINX_BUILD = "bindings/pydrake/doc/sphinx_build.py"


def die(s):
    print(s, file=sys.stderr)
    exit(1)


def main():
    assert isfile(_SPHINX_BUILD), "Please execute via 'bazel run'"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out_dir", type=str, required=True)
    parser.add_argument(
        "--debug", action="store_true")
    args = parser.parse_args()
    out_dir = args.out_dir
    if not isabs(out_dir):
        die("--out_dir must be absolute path: {}".format(out_dir))
    if isdir(out_dir):
        die("--out_dir must not exist: {}".format(out_dir))
    mkdir(out_dir)

    input_dir = dirname(abspath(__file__))
    tmp_dir = join(out_dir, "_tmp")
    doctree_dir = join(tmp_dir, "doctrees")
    src_dir = join(tmp_dir, "src")
    mkdir(tmp_dir)
    # Symlink inputs to src dir (so that we can also generate doc modules).
    mkdir(src_dir)
    for f in listdir(input_dir):
        src_f = join(src_dir, f)
        symlink(join(input_dir, f), src_f)
    # Generate doc modules.
    refresh_doc_modules(output_dir=src_dir)
    print("Generating documentation...")
    check_call([
        sys.executable, _SPHINX_BUILD,
        "-b", "html",  # HTML output.
        "-a", "-E",  # Don't use caching.
        "-d", doctree_dir,
        "-N", "-Q",  # Be very quiet.
        "-T",  # Traceback.
        src_dir,  # Source dir.
        out_dir,
    ])
    if not args.debug:
        rmtree(tmp_dir)
    else:
        print("DEBUG: Temporary files: {}".format(tmp_dir))
    print("Documentation: file://{}".format(join(out_dir, "index.html")))


if __name__ == "__main__":
    main()
