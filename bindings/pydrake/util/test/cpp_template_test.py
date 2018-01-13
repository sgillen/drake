#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

import unittest

import pydrake.util.cpp_template as m

class A(object):
    pass


class B(object):
    pass


class C(object):
    pass


def func_a():
    return 1


def func_b():
    return 2


def func_c(self):
    return (self, 3)


def func_d(self):
    return (self, 4)


class TestCppTemplate(unittest.TestCase):
    def test_base(self):
        tpl = m.Template("BaseTpl")
        self.assertEquals(str(tpl), "<Template '__main__.BaseTpl'>")

        # Single arguments.
        tpl.add_instantiation(int, 1)
        self.assertEquals(tpl[int], 1)
        self.assertEquals(tpl.get_instantiation(int), (1, (int,)))
        self.assertEquals(tpl.get_param_list(1) == [(int,)])
        # Duplicate parameters.
        self.assertRaises(
            lambda: tpl.add_instantiation(int, 4),
            RuntimeError)
        # Invalid parameters.
        self.assertRaises(lambda: tpl[float], RuntimeError)
        # New instantiation.
        tpl.add_instantiation(float, 2)
        self.assertEquals(tpl[float], 2)

        # Default instantiation.
        self.assertEquals(tpl[None], 1)
        self.assertEquals(tpl.get_instantiation(), (1, (int,)))

        # Multiple arguments.
        tpl.add_instantiation((int, int), 3)
        self.assertEquals(tpl[int, int], 3)
        # Duplicate instantiation.
        self.add_instantiation((float, float), 1)
        self.assertEquals(tpl.get_param_list(1), [(int,), (float, float)])

        # List instantiation.
        def instantiation_func(param):
            return 100 + len(param)
        func_a = (str,) * 5
        func_b = (str,) * 10
        self.add_instantiations(instantiation_func, [func_a, func_b])
        self.assertEquals(tpl[func_a] == 105)
        self.assertEquals(tpl[func_b] == 110)

    def test_class(self):
        tpl = m.TemplateClass("ClassTpl")

        tpl.add_instantiation(int, A)
        tpl.add_instantiation(float, B)

        self.assertEquals(tpl[int], A)
        self.assertEquals(str(A), "<class '__main__.ClassTpl[int]'>")
        self.assertEquals(tpl[float], B)
        self.assertEquals(str(A), "<class '__main__.ClassTpl[float]'>")

    def test_function(self):
        tpl = m.TemplateFunction("func")

        tpl.add_instantiation(int, func_a)
        tpl.add_instantiation(float, func_b)

        self.assertEquals(tpl[int](), 1)
        self.assertEquals(tpl[float](), 2)
        self.assertEquals(str(tpl), "<TemplateFunction '__main__.func'>")

    def test_method(self):
        C.method = TemplateMethod("method", C)
        C.method.add_instantiation(int, func_c)
        C.method.add_instantiation(float, func_d)

        self.assertEquals(str(C.method), "<unbound TemplateMethod 'C.method'>")
        self.assertEquals(C.method[int], func_c)
        self.assertEquals(str(C.method[int]), "<unbound method 'func_c'>")
        self.assertEquals(C.method[int](None), (None, 3))
        self.assertEquals(C.method[float](None), (None, 4))

        c = C()
        self.assertTrue(
            str(c.method).startswith("<bound TemplateMethod 'C.method' of "))
        self.assertTrue(
            str(c.method[int]).startswith("<bound method 'func_c' of "))
        self.assertEquals(c.method[int](), (c, 3))
        self.assertEquals(c.method[float](), (c, 4))


if __name__ == '__main__':
    unittest.main()
