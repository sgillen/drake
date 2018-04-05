"""
Provides extensions for containers of Drake-related objects.
"""


class _EqualityProxy(object):
    # TODO(eric.cousineau): Copy input object?
    def __init__(self, value):
        self._value = value

    def _get_value(self):
        return self._value

    def __hash__(self):
        return hash(self._value)

    def __eq__(self, other):
        return type(self) == type(other) and hash(self) == hash(other)

    def __nonzero__(self):
        return bool(self._value)

    value = property(_get_value)


class _DictKeyWrap(dict):
    # Wraps a dictionary's key access.
    def __init__(self, tmp, key_wrap, key_unwrap):
        dict.__init__(self)
        self._key_wrap = key_wrap
        self._key_unwrap = key_unwrap
        for key, value in tmp.iteritems():
            self[key] = value

    def __setitem__(self, key, value):
        return dict.__setitem__(self, self._key_wrap(key), value)

    def __getitem__(self, key):
        return dict.__getitem__(self, self._key_wrap(key))

    def __delitem__(self, key):
        return dict.__delitem__(self, self._key_wrap(key))

    def __contains__(self, key):
        return dict.__contains__(self, self._key_wrap(key))

    def items(self):
        return zip(self.keys(), self.values())

    def keys(self):
        return map(self._key_unwrap, dict.keys(self))

    def iterkeys(self):
        # Non-performant, but sufficient for now.
        return self.keys()

    def iteritems(self):
        # Non-performant, but sufficient for now.
        return self.items()


class EqualityProxyDict(_DictKeyWrap):
    """Implements a dictionary where entries are keyed only by hash value.

    By default, equality is based on type and hash. This may be overridden using
    the `eq` kwarg.
    """
    def __init__(self, *args, **kwargs):
        if "eq" not in kwargs:
            cls = _EqualityProxy
        else:
            eq = kwargs.pop("eq")

            class Proxy(_EqualityProxy):
                def __eq__(self, other):
                    return eq(self.value, other.value)

            cls = Proxy
        tmp = dict(*args, **kwargs)
        _DictKeyWrap.__init__(self, tmp, cls, cls.value)


class EqualToDict(EqualityProxyDict):
    """Implements a dictionary where keys are compared using `lhs.EqualTo(rhs)`,
    where `lhs` and `rhs` are of compatible types.
    """
    def __init__(self, *args, **kwargs):
        eq = lambda lhs, rhs: lhs.EqualTo(rhs)
        EqualityProxyDict.__init__(self, *args, eq=eq, **kwargs)
