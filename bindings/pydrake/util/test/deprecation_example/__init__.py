import sys
from pydrake.util.deprecation import (
    ModuleShim,
    deprecated,
    deprecated_attribute,
)

value = 1
deprecated_value = deprecated_attribute("Bad value")(100)
sub_module = property(
    lambda obj: "This would be a shim",
    # N.B. We must implement a `setter` for values that have submodule aliases.
    # This is called when the submodule is imported.
    # N.B. As this is written, this will overwrite the original descriptor,
    # which is fine.
    lambda obj, value: setattr(obj, "sub_module", value))


def _handler(name):
    raise AttributeError()


__all__ = ["value", "sub_module"]
ModuleShim.install(__name__, _handler)


class ExampleClass(object):
    doc_method = "Method Doc"
    doc_prop = "Prop Doc"
    message_method = "`deprecated_method` is deprecated"
    message_prop = "`deprecated_prop` is also deprecated"
    message_attr = "`deprecated_attr` is deprecated, too"

    # Deprecate method.
    @deprecated(message_method)
    def deprecated_method(self):
        """Method Doc"""
        return 1

    # Deprecate public property.
    _deprecated_prop = property(lambda self: 2, doc=doc_prop)
    deprecated_prop = deprecated(message_prop)(_deprecated_prop)
    deprecated_attr = deprecated_attribute(message_attr)(10)
