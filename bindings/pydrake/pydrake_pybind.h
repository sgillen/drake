#pragma once

#include <pybind11/pybind11.h>

namespace drake {
namespace pydrake {

/**
@page python_bindings Python Bindings

# Conventions

## Target Conventions

Target names should be of the following form:

*   `*_py`

    A Python library (can be pure Python or pybind)
    File Names: `*.py`, `*_py.cc`

* `*_pybind`

    A C++ library for adding pybind-specific utilities to be consumed by C++.
    File Names: `*_pybind.{h,cc}`

File names should follow form with their respective target.

For Bazel: Given that `libdrake.so` relies on static linking for components,
any common headers should be robust against ODR violations. This can
be normally be achieved by using header-only libraries.
If singletons are required (e.g. for `util/cpp_param_pybind`), consider storing
the singleton values using Python.

## pybind Module Definitions

* Any Drake pybind module should include this header file, `pydrake_pybind.h`.

* `PYBIND_MODULE` should be used to define modules.

* Modules should be defined within the namespace `drake::pydrake`.

* The alias `namespace py = pybind11` is defined as `drake::pydrake::py`. Drake
modules should not re-define this alias at global scope.

* If a certain namespace is being bound (e.g. `drake::systems::sensors`), you
may use `using namespace drake::systems::sensors` within functions or
anonymous namespaces. Avoid `using namespace` directives otherwise.

# Keep Alive Behavior

`py::keep_alive` is used heavily throughout this code. For more
information, please see:
http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive

Terse notes are added to method bindings to indicate the patient
(object being kept alive by nurse) and the nurse (object keeping patient
alive). To expand on them:
* "Keep alive, ownership" implies that one argument is owned directly by
one of the other arguments (`self` is included in those arguments, for
`py::init<>` and class methods).
* "Keep alive, reference" implies a reference that is lifetime-sensitive
(something that is not necessarily owned by the other arguments).
* "Keep alive, transitive" implies a transfer of ownership of owned
objects from one container to another. (e.g. transfering all `System`s
from `DiagramBuilder` to `Diagram` when calling
`DiagramBuilder.Build()`.)

*/

// TODO(eric.cousineau): Add API naming conventions (#7819).

/// @defgroup Convenience aliases
/// @{

/// Shorthand alias to `pybind` for consistency.
/// @note Downstream users should avoid `using namespace drake::pydrake`, as
/// this may create ambiguous aliases (especially for GCC). Instead, consider
/// an alias.
namespace py = pybind11;

/// Used when returning `T& or `const T&`, as pybind's default behavior is to
/// copy lvalue references.
const auto py_reference_internal =
    py::return_value_policy::reference_internal;

/// Used when returning references to objects that are internally owned by
/// `self`. Implies both `py_reference` and `py::keep_alive<0, 1>`, which
/// implies "Keep alive, reference: `return` keeps` self` alive".
const auto py_reference = py::return_value_policy::reference;

/// @}

// TODO(eric.cousineau): pybind11 defaults to C++-like copies when dealing
// with rvalues. We should wrap this into a drake-level binding, so that we
// can default this to `py_reference` or `py_reference_internal.`

}  // namespace pydrake
}  // namespace drake
