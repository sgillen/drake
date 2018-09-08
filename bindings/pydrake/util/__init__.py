# Expose all modules at the time of transition.
# TODO(eric.cousineau): Add deprecation message.
import sys

from pydrake.common import (
    compatibility,
    containers,
    cpp_const,
    cpp_param,
    cpp_template,
    deprecation,
    eigen_geometry,
)

def _alias_modules(names):
    # Alias registered modules.
    for name in names:
        m = globals()[name]
        sys.modules["pydrake.util." + name] = m

_alias_modules([
    "compatibility",
    "containers",
    "cpp_const",
    "cpp_param",
    "cpp_template",
    "deprecation",
    "eigen_geometry",
])
