"""Provides utilities to aid in scalar type conversion."""

import copy

from pydrake.systems.framework import (
    LeafSystem_,
    SystemScalarConverter,
)
from pydrake.util.cpp_template import TemplateClass


def _get_conversion_pairs(T_list):
    # Intersect.
    T_compat = []
    for T in T_list:
        if T in SystemScalarConverter.SupportedScalars:
            T_compat.append(T)
    # Outer join without duplicates.
    T_pairs = []
    for T in T_compat:
        for U in T_compat:
            if T != U:
                T_pairs.append((T, U))
    return T_pairs


def _patch_system_init(template, T, instantiation):
    # Check that the user has not defined `__init__`, nad has defined
    # `_construct` and `_construct_copy`.
    if not issubclass(instantiation, LeafSystem_[T]):
        raise RuntimeError(
            "{} must inherit from LeafSystem_[T] (encountered for T={})"
            .format(instantiation, T.__name__))
    # Use the immediate `__dict__`, rather than querying the attributes, so
    # that we don't get spillover from inheritance.
    d = instantiation.__dict__
    no_init = "__init__" not in d
    has_custom_init = ("_construct" in d) and ("_construct_copy" in d)
    if not (no_init and has_custom_init):
        raise RuntimeError(
            "Convertible systems should not define `__init__`, but must define "
            "`_construct` and `_construct_copy` instead.")

    def system_init(self, *args, **kwargs):
        converter = None
        if "converter" in kwargs:
            converter = kwargs.pop("converter")
        if converter is None:
            # Use default converter.
            converter = template._converter
        if _check_if_copying(template, self, *args, **kwargs):
            instantiation._construct_copy(
                self, *args, converter=converter, **kwargs)
        else:
            instantiation._construct(
                self, *args, converter=converter, **kwargs)

    instantiation.__init__ = system_init


def define_convertible_system(name, T_list=None, T_pairs=None):
    """Provides a decorator which can be used define a scalar-type convertible
    System.

    @param T_list
        List of T's that the given system supports. By default, it is all types
        supported by `LeafSystem`.
    @param template TemplateClass instance.
    @param T_pairs List of pairs, (T, U), defining a conversion from a
        scalar type of U to T.
        If None, this will use all possible pairs that the Python bindings
        of `SystemScalarConverter` support.
    """
    if T_list is None:
        T_list = SystemScalarConverter.SupportedScalars
    param_list = []
    for T in T_list:
        assert T in SystemScalarConverter.SupportedScalars, (
            "Type {} is not a supported scalar type".format(T))
        param_list.append((T,))
    if T_pairs is None:
        T_pairs = _get_conversion_pairs(T_list)
    for T_pair in T_pairs:
        assert T_pair in \
            SystemScalarConverter.SupportedConversionPairs, (
            "Conversion {} is not supported".format(T_pair))

    def decorator(instantiation_func):

        @TemplateClass.define(name, param_list=param_list)
        def template(param):
            T, = param
            instantiation = instantiation_func(T)
            # Check and patch the class.
            _patch_system_init(template, T, instantiation)

            return instantiation

        # Tack on converer for ease of testing.
        template._converter = _make_converter(template, T_pairs)
        return template

    return decorator


# Global cache.
_converter_cache = {}


def _check_if_copying(template, obj, *args, **kwargs):
    # Checks if a function signature implies a copy constructor.
    if len(args) >= 1:
        if template.is_subclass_of_instantiation(type(args[0])):
            return True
    return False


def _make_converter(template, T_pairs):
    # Creates system scalar converter for the template class.
    cache_key = (template, tuple(T_pairs))
    cache_entry = _converter_cache.get(cache_key)
    if cache_entry:
        return copy.copy(cache_entry)

    # Generate and register each conversion.
    converter = SystemScalarConverter()

    # Define capture to ensure the current values are bound, and do not
    # change through iteration.
    # N.B. This does not directly instantiation the template; it is deferred to
    # when the conversion is called.
    def add_captured(T_pair):
        T, U = T_pair

        def conversion(system):
            assert isinstance(system, template[U])
            return template[T](system)

        converter.Add[T, U](conversion)

    map(add_captured, T_pairs)

    _converter_cache[cache_key] = copy.copy(converter)
    return converter
