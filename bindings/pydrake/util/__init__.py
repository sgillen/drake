"""
Backwards compatibility for symbols moved to `pydrake.common`.
"""
import importlib
import sys

# TODO(eric.cousineau): Add deprecation message on 2018/11/01.


def _alias(new, old=None):
    if old is None:
        old = new
    m = importlib.import_module("pydrake.common." + new)
    sys.modules["pydrake.util." + old] = m


__all__ = []

_alias("compatibility")
_alias("containers")
_alias("_cpp_const", "cpp_const")
_alias("_cpp_param", "cpp_param")
_alias("_cpp_template", "cpp_template")
_alias("deprecation")
_alias("eigen_geometry")
