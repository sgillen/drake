import copy


def check_copy(f, obj):
    """Checks copy function `f` to ensure `obj` is equal to its copy, and that
    it is not the same instance."""
    obj_copy = f(obj)
    return obj == obj_copy and obj is not obj_copy
