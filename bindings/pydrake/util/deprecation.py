"""
Provides deprecation warnings and utilities for triggering warnings.

By default, this sets all `DrakeDeprecationWarnings` to be shown `"once"`,
which overrides any `-W` command-line arguments. To change this behavior, you
can do something like:

>>> import warnings
>>> from pydrake.util.deprecation import DrakeDeprecationWarning
>>> warnings.simplefilter("always", DrakeDeprecationWarning)

If you would like to disable all Drake-related warnings, you may use the
`"ignore"` action for `warnings.simplefilter`.
"""

import sys
import traceback
import warnings

# TODO(eric.cousineau): Make autocomplete ignore `ModuleShim` attributes
# (e.g. `install`).


class ModuleShim(object):
    """Provides a shim for automatically resolving extra variables, and
    respects descriptors within a module.

    This can be used to deprecate import alias in modules to simplify
    dependencies, or deprecate aliases which already exist.

    @see https://stackoverflow.com/a/7668273/7829525
    """
    # TODO(eric.cousineau): Consider overriding `__dict__` if need be.

    def __init__(self, orig_module):
        # assert hasattr(orig_module, "__all__"), (
        #     "Please define `__all__` for this module.")
        # https://stackoverflow.com/a/16237698/7829525
        object.__setattr__(self, '_orig_module', orig_module)

    def __getattr__(self, name):
        # Use the original module if possible.
        m = self._orig_module
        value = getattr(m, name)
        if _is_descriptor(value):
            return value.__get__(m, type(m))
        else:
            return value

    def __setattr__(self, name, value):
        # Redirect writes to the original module.
        m = self._orig_module
        desc = self._get_descriptor(name)
        if desc:
            desc.__set__(m, value)
        else:
            setattr(m, name, value)

    def __delattr__(self, name):
        # Redirect deletions to the original module.
        m = self._orig_module
        desc = self._get_descriptor(name)
        if desc:
            desc.__del__(m)
        else:
            delattr(m, name)

    def __repr__(self):
        return repr(self._orig_module)

    def _get_descriptor(self, name):
        m = self._orig_module
        value = getattr(m, name, None)
        if _is_descriptor(value):
            return value
        return None

    @classmethod
    def install(cls, name):
        """
        Hook into module's attribute accessors and mutators, and permit a module to take descriptors.

        @param name
            Module name. Generally should be __name__.
        """
        old_module = sys.modules[name]
        new_module = cls(old_module)
        print(sys.modules.keys())
        sys.modules[name] = new_module

        # TODO(eric.cousineau): Use `meta_path` or something to handle wrapping
        # C extensions...
        # http://xion.org.pl/2012/05/06/hacking-python-imports/


class DrakeDeprecationWarning(DeprecationWarning):
    """Extends `DeprecationWarning` to permit Drake-specific warnings to
    be filtered by default, without having side effects on other libraries."""
    addendum = ("\n    Please see `help(pydrake.util.deprecation)` " +
                "for more information.")

    def __init__(self, message, *args):
        extra_message = message + DrakeDeprecationWarning.addendum
        DeprecationWarning.__init__(self, extra_message, *args)


def _is_descriptor(value):
    if value is None:
        return False
    getter = getattr(value, "__get__", None)
    if getter is not None:
        return True
    return False


class _ConstantDescriptor(object):
    def __init__(self, value):
        self._value = value

    def __get__(self, obj, objtype):
        return self._value

    def __set__(self, obj, value):
        raise Exception("Read-only value")

    def __delete__(self, obj):
        raise Exception("Read-only value")


def _deprecate_attribute(cls, name, message):
    # Deprecates an attribute which is directly owned by an object.
    # TODO(eric.cousineau): Permit a non-constant value wrapper version if need
    # be.
    original = getattr(cls, name)
    if not _is_descriptor(original):
        # N.B. We don't use `property` here because, for some reason, I (Eric)
        # am unable to get it to be called.
        value = _ConstantDescriptor(original)
    else:
        value = original
    setattr(cls, name, deprecated(message)(value))


def deprecated_attribute(message):
    def wrapped(original):
        assert not _is_descriptor(original)
        value = _ConstantDescriptor(original)
        return _DeprecatedDescriptor(value, message)
    return wrapped


def _warn_deprecated(message, stacklevel=2):
    # Logs a deprecation warning message.  Also used by `deprecation_pybind.h`
    # in addition to this file.
    warnings.warn(
        message, category=DrakeDeprecationWarning, stacklevel=stacklevel)


class _DeprecatedDescriptor(object):
    """Wraps a descriptor to warn that it is deprecated any time it is
    acccessed."""

    def __init__(self, original, message):
        assert hasattr(original, '__get__'), "`original` must be a descriptor"
        self._original = original
        self.__doc__ = self._original.__doc__
        self._message = message

    def _warn(self):
        _warn_deprecated(self._message, stacklevel=4)

    def __get__(self, obj, objtype):
        self._warn()
        return self._original.__get__(obj, objtype)

    def __set__(self, obj, value):
        self._warn()
        self._original.__set__(obj, value)

    def __delete__(self, obj):
        self._warn()
        self._original.__delete__(obj)


def deprecated(message):
    """Decorator that deprecates a member of a class based on access.

    @param message Warning message when member is accessed.

    @note This differs from other implementations in that it warns on
    access, not when the method is called. For other methods, see
    the examples in https://stackoverflow.com/a/40301488/7829525.

    Use `ModuleShim` for deprecating variables in a module."""
    def wrapped(original):
        return _DeprecatedDescriptor(original, message)

    return wrapped


def install_numpy_warning_filters(force=False):
    """Install warnings filters specific to NumPy."""
    global installed_numpy_warning_filters
    if installed_numpy_warning_filters and not force:
        return
    installed_numpy_warning_filters = True
    # Warnings specific to comparison with `dtype=object` should be raised to
    # errors (#8315, #8491). Without them, NumPy will return effectively
    # garbage values (e.g. comparison based on object ID): either a scalar bool
    # or an array of bools (based on what objects are present and the NumPy
    # version).
    # N.B. Using a `module=` regex filter does not work, as the warning is
    # raised from C code, and thus inherits the calling module, which may not
    # be "numpy\..*" (numpy/numpy#10861).
    warnings.filterwarnings(
        "error", category=DeprecationWarning, message="numpy equal will not")
    warnings.filterwarnings(
        "error", category=DeprecationWarning,
        message="elementwise == comparison failed")


warnings.simplefilter('once', DrakeDeprecationWarning)
installed_numpy_warning_filters = False
