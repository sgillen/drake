from __future__ import absolute_import, print_function

import ctypes
import numpy as np

"""
@file
Defines a mapping between Python and C++ types, and provides canonical Python
types as they relate to C++.
"""

# Define these first, as they are used in `cpp_types_py.cc`
# (transitively, `cpp_types_pybind.cc`).


def _get_type_name(t):
    # Gets type name as a string.
    prefix = t.__module__ + "."
    if prefix == "__builtin__.":
        prefix = ""
    return prefix + t.__name__


class _StrictMap(object):
    # Provides a map which may only add a key once, and ensures that literal
    # types are unique when needed (e.g. `1` and `True` should be distinct).
    def __init__(self):
        self._values = dict()

    def _strict_key(self, key):
        # Ensures keys are strictly scoped to the values (for literals).
        return (type(key), key)

    def add(self, key, value):
        skey = self._strict_key(key)
        assert skey not in self._values, "Already added: {}".format(skey)
        self._values[skey] = value

    def get(self, key, default):
        skey = self._strict_key(key)
        return self._values.get(skey, default)


# Load and import type registry.
from ._cpp_types_py import _type_registry  # noqa


def get_types_canonical(param):
    """Gets the canonical types for a set of Python types (canonical as in
    how they relate to C++ types. """
    return tuple(map(_type_registry.GetPyTypeCanonical, param))


def get_type_names(param):
    """Gets the canonical type names for a set of Python types (canonical as in
    how they relate to C++ types. """
    return tuple(map(_type_registry.GetName, param))
