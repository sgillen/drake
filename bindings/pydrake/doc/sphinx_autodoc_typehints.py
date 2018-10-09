# http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter
from __future__ import print_function

from collections import namedtuple
import re

from sphinx.ext import autodoc

from pydrake.util.cpp_template import TemplateBase

import sys
sys.stdout = sys.stderr

#: extended signature RE: with explicit module name separated by ::
class IrregularExpression(object):
    FakeMatch = namedtuple('FakeMatch', 'groups')
    py_sig_func = re.compile(
        r'''^ \s* (?: \((.*)\)       # optional: arguments
                  (?:\s* -> \s* (.*))?  #           return annotation
                  )? $                   # and nothing more
              ''', re.VERBOSE)

    def match(self, s):
        s_orig = s
        # explicit_modname
        explicit_modname = None
        if "::" in s:
            pieces = s.split("::")
            explicit_modname = '::'.join(pieces[:-1] + [''])
            s = pieces[-1]
        pieces = []
        piece = ''
        num_open = 0
        i = 0
        for c in s:
            if c.isspace() or c == '(':
                break
            if num_open == 0 and c == '.':
                pieces.append(piece)
                piece = ''
            else:
                if c == '[':
                    num_open += 1
                elif c == ']':
                    num_open -= 1
                piece += c
            i += 1
        pieces.append(piece)
        m = self.py_sig_func.match(s[i:])
        if m is None:
            return None
        func_groups = m.groups()
        path = '.'.join(pieces[:-1]) or None
        if path:
            path += "."
        else:
            return None
        base = pieces[-1]
        groups = lambda: (explicit_modname, path, base) + func_groups
        if "[" not in s_orig:
            m = old.match(s_orig)
            try:
                assert m is not None, (s_orig, groups())
                old_groups = m.groups()
                assert old_groups == groups(), s_orig
            except:
                import traceback
                traceback.print_stack()
                raise
        # print(("match", s_orig, groups()))
        return self.FakeMatch(groups)

old = autodoc.py_ext_sig_re
autodoc.py_ext_sig_re = IrregularExpression()


class TemplateDocumenter(autodoc.ModuleLevelDocumenter):
    """
    Specialized Documenter subclass for templates.
    """
    objtype = 'template'
    member_order =  autodoc.ClassDocumenter.member_order
    directivetype = 'class'
    option_spec = {}

    @classmethod
    def can_document_member(cls, member, membername, isattr, parent):
        return isinstance(member, TemplateBase)

    def get_object_members(self, want_all):
        members = []
        print(self.object)
        for param in self.object.param_list:
            instantiation = self.object[param]
            members.append((instantiation.__name__, instantiation))
            print(" - ", param)
        return False, members

    def import_object(self):
        out = autodoc.ModuleLevelDocumenter.import_object(self)
        # print("import_object: ", out)
        self._DBG = True
        return out

    def check_module(self):
        return True
        # out = autodoc.ModuleLevelDocumenter.check_module(self)
        # print("check_module: ", out)
        # return out

    # def generate(self, **kwargs):


def tpl_getter(obj, name, *defargs):
    if "[" in name:
        assert name.endswith(']'), name
        for param in obj.param_list:
            inst = obj[param]
            if inst.__name__ == name:
                return inst
        raise RuntimeError("Not a template?")
    return autodoc.safe_getattr(obj, name, *defargs)


def setup(app):
    app.add_autodoc_attrgetter(object, tpl_getter)
    app.add_autodocumenter(TemplateDocumenter)
    return dict(parallel_read_safe=True)


# # from pydrake.all import Adder_
# import pydrake.all
# Adder_ = autodoc.safe_getattr(pydrake.all, 'Adder_')
# print(Adder_)
# exit(10)
