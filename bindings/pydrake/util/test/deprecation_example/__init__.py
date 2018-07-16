from pydrake.util.deprecation import (
    ModuleShim,
    deprecated,
    _deprecate_attribute,
)

value = 1


def _handler(name):
    if name == "sub_module":
        return "This would be a shim"
    else:
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

    deprecated_attr = 10


_deprecate_attribute(ExampleClass, "deprecated_attr", ExampleClass.message_attr)
