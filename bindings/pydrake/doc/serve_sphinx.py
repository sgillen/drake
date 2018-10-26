"""
This program serves sphinx.zip for a web browser.

Run this via:
  $ bazel run //doc:serve_sphinx
"""

from __future__ import print_function

import argparse
import os
from os.path import abspath, isdir, join
from shutil import rmtree
import subprocess
import sys
import webbrowser


def str2bool(value):
    # From: https://stackoverflow.com/a/19233287/7829525
    return value.lower() in ("yes", "y", "true", "t", "1")


parser = argparse.ArgumentParser()
parser.add_argument(
    "--out_dir", type=str, default=None)
parser.register('type', 'bool', str2bool)
parser.add_argument(
    "--browser", type='bool', default=True, metavar='BOOL',
    help="Open browser. Disable this if you are frequently recompiling.")
parser.add_argument(
    "--port", type=int, default=8001, metavar='PORT',
    help="Port for serving doc pages with a HTTP server.")
args = parser.parse_args()

out_dir = args.out_dir
if out_dir is None:
    out_dir = abspath("sphinx-tmp")
    if isdir(out_dir):
        rmtree(out_dir)

# Generate documentation.
# N.B. This prints out a link to the documentation.
subprocess.check_call([
    sys.executable, "bindings/pydrake/doc/gen_sphinx.py",
    "--out_dir", out_dir])

# Try the default browser.
if args.browser:
    print("Opening webbrowser", file=sys.stderr)
    webbrowser.open("file://{}".format(join(out_dir, "index.html")))
