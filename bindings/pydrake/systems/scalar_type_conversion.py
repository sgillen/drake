"""Provides simple utilities to aid in scalar type conversion."""

import copy

from pydrake.systems.framework import SystemScalarConverter
from pydrake.util.cpp_template import is_instantiation_of


def _to_scalar(item):
    assert len(item) == 1
    return item[0]


_converter_cache = {}


def create_system_scalar_converter(template, param_pairs=None, use_cache=True):
    """Creates system scalar converter for a given template, assuming that the
    System class has a scalar-type-converting copy constructing.
    @see check_system_copy_constructor

    @param template TemplateClass instance.
    @param param_pairs List of pairs, (T, U), where 
        If None, this will use all possible pairs that the Python bindings of
        `SystemScalarConverter` supports.
    @param use_cache Use cache from a global store, returning a copy on a cache
    hit.
    """

    cache_key = (template, param_pairs)
    if use_cache:
        cache_entry = _converter_cache.get(cache_key)
        if cache_entry:
            return copy.copy(cache_entry)

    if param_pairs is None:
        param_list = template.param_list
        param_pairs = []

        # Flatten.
        param_flat = map(to_scalar, param_list)
        # Intersect.
        param_compat = [
            param in param_flat
            if param in SystemScalarConverter.SupportedScalars]
        # Outer join without duplicates.
        for T in param_compat:
            for U in param_compat:
                if T != U:
                    param_pairs.append((T, U))

    # Generate and register each conversion.
    converter = SystemScalarConverter()
    for T, U in param_pairs:

        def conversion(system):
            assert isinstance(system, template[U])
            return template[T](system)

        converter.Add[T, U](conversion)

    if use_cache:
        _converter_cache[cache_key] = copy.copy(converter)
    return converter


def check_system_copy_constructor(template, obj, args, kwargs, alt_init=None):
    """Determines if constructor call has one parameter which is an
    instantiation of the current template, indicating that this is a System
    scalar type conversion copy constructor.

    @param template TemplateClass instance.s
    @param obj Instance of current object (`self`)
    @param args Positional arguments to function.
    @param kwargs Keyword arguments to constructor.
    @param alt_init Alternative constructor, if not a copy constructor.
        If supplied, then this will be called with the arguments.
    """
    if len(args) == 1 and len(kwargs) == 0:
        if is_instantiation_of(type(obj), template):
            return True
    if alt_init:
        alt_init(obj, *args, **kwargs)
    return False
