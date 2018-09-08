# Ensure we can import symbols from old paths.
from pydrake.util.compatibility import maybe_patch_numpy_formatters
from pydrake.util.containers import EqualToDict
from pydrake.util.cpp_const import ConstError
from pydrake.util.cpp_param import get_param_canonical
from pydrake.util.cpp_template import TemplateBase
from pydrake.util.deprecation import DrakeDeprecationWarning
from pydrake.util.eigen_geometry import Quaternion

print("Done")
