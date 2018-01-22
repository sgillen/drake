#pragma once

#include <pybind11/pybind11.h>

namespace drake {
namespace pydrake {

/**
@page Python Bindings

@tableofcontents

@section Keep Alive Behavior
`py::keep_alive` is used heavily throughout this code. For more
information, please see:
http://pybind11.readthedocs.io/en/stable/advanced/functions.html#keep-alive

Terse notes are added to method bindings to indicate the patient
(object being kept alive by nurse) and the nurse (object keeping patient
alive).
* "Keep alive, ownership" implies that one argument is owned directly by
one of the other arguments (`self` is included in those arguments, for
`py::init<>` and class methods).
* "Keep alive, reference" implies a reference that is lifetime-sensitive
(something that is not necessarily owned by the other arguments).
* "Keep alive, transitive" implies a transfer of ownership of owned
objects from one container to another. (e.g. transfering all `System`s
from `DiagramBuilder` to `Diagram` when calling
`DiagramBuilder.Build()`.)

@note `py::return_value_policy::reference_internal` implies
`py::keep_alive<0, 1>`, which implies "Keep alive, reference: `return` keeps
`self` alive".

The following convenience aliases are provided:
* `py_reference` is used when `keep_alive` is explicitly used (e.g. for extraction methods, like `GetMutableSubsystemState`).
* `py_reference_internal` is used when pointers / lvalue references are
returned (no need for `keep_alive`, as it is implicit.
*/

/** @defgroup Convenience aliases
@subsection 
* `py_reference` is used when `keep_alive` is explicitly used (e.g. for extraction methods, like `GetMutableSubsystemState`).
`py_reference_internal` is used when pointers / lvalue references are returned (no need
for `keep_alive`, as it is implicit.
const auto py_reference_internal =
    pybind11::return_value_policy::reference_internal;

// TODO(eric.cousineau): pybind11 defaults to C++-like copies when dealing
// with rvalues. We should wrap this into a drake-level binding, so that we
// can default this to `reference` or `reference_internal.`

const auto py_reference = pybind11::return_value_policy::reference;

}  // namespace pydrake
}  // namespace drake
