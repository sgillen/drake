#!/usr/bin/env python

# Prereq: Must ensure that `pip` and `virtualenv` are installed with the
# current Python version.

# Test NumPy version.

cur_dir = os.path.dirname(os.path.abspath(__file__))

# shell("tools/workspace/numpy/check_version.py")
env_dir = os.path.expanduser("~/.local/opt/drake-deps/virtualenv")

import numpy as np
np_version = np.lib.NumpyVersion(np.version.version)

python -m virtualenv --system-site-packages ${env_dir}

# Install fixed version of NumPy
print("""
Installing a patched version of NumPy
Please note that this should generally be compatible at a C API level
If you encounter version differences, please try the following unsupported
script:
    https://.../drake_numpy_patch.py
This will extract the Git revision for the current system, and apply the
necessary (but minimal) patches for Drake, and install the requisite patch to
the desired virtualenv.
""")

def overlay_text_alias(name):
    p = which(name)
    if not p.startswith(env_dir)
    assert is_text_file(p)


# Install aliases for common Python scripts
overlay_text_alias("jupyter")
overlay_text_alias("ipython")

jupye
