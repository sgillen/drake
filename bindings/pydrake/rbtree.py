"""`pydrake.rbtree` is deprecated. Please use
`pydrake.multibody.rigid_body_tree` and `pydrake.multibody.parsers` instead.
This module will be removed after 10/15/2018.
"""

from __future__ import absolute_import

import warnings

from .util.deprecation import DrakeDeprecationWarning
from .multibody.rigid_body_tree import *
from .multibody.parsers import PackageMap

# N.B. `stacklevel` is a bit difficult here, because it could be triggered by
# `import pydrake.rbtree` (module import) or `from pydrake import rbtree`
# (ModuleShim).
warnings.warn(__doc__, category=DrakeDeprecationWarning)

# TODO(eric.cousineau): remove these direct imports and leave the constants
# inside the FloatingBaseType enum class (#4950).
kFixed = FloatingBaseType.kFixed
kRollPitchYaw = FloatingBaseType.kRollPitchYaw
kQuaternion = FloatingBaseType.kQuaternion
