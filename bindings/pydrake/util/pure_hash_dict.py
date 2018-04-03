class PureHashProxy(object):
    # TODO: Copy input object?
    def __init__(self, value):
        self._value = value

    def _get_value(self):
        return self._value

    def __hash__(self):
        return hash(self._value)

    def __eq__(self, other):
        return hash(self._value) == hash(other)

    def __nonzero__(self):
        return bool(self._value)

    value = property(_get_value)


class DictWrap(dict):
    def __init__(self, tmp, key_wrappers):
        dict.__init__(self)
        self._key_wrap, self._key_unwrap = key_wrappers
        for key, value in tmp.iteritems():
            self[key] = value

    def __setitem__(self, key, value):
        return dict.__setitem__(self, self._key_wrap(key), value)

    def __getitem__(self, key):
        return dict.__getitem__(self, self._key_wrap(key))

    def __delitem__(self, key):
        return dict.__delitem__(self, self._key_wrap(key))

    def items(self):
        return zip(self.keys(), self.values())

    def keys(self):
        return map(self._key_unwrap, dict.keys(self))

    def iterkeys(self):
        return self.keys()

    def iteritems(self):
        # For now, just return everything.
        return self.items()


class PureHashDict(DictWrap):
    def __init__(self, *args, **kwargs):
        tmp = dict(*args, **kwargs)
        DictWrap.__init__(
            self, tmp, key_wrappers=(PureHashProxy, PureHashProxy.value))
