# http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter
from __future__ import print_function

from collections import namedtuple
import re

from sphinx.ext import autodoc

from pydrake.util.cpp_template import TemplateBase

# import sys
# sys.stdout = sys.stderr

def rindex(s, sub):
    return len(s) - s[::-1].index(sub) - len(sub)

#: extended signature RE: with explicit module name separated by ::
class IrregularExpression(object):
    FakeMatch = namedtuple('FakeMatch', 'groups')
    py_sig = re.compile(
        r'''^     (\w.*?) \s*               # symbol
                  (?:
                      \((.*)\)              # optional: arguments
                      (?:\s* -> \s* (.*))?  # return annotation
                  )? $
              ''', re.VERBOSE)

    def match(self, full):
        m = self.py_sig.match(full)
        if not m:
            return None
        s, arg, retann = m.groups()
        # Extract module name using a greedy match.
        explicit_modname = None
        if "::" in s:
            pos = rindex(s, "::") + 2
            explicit_modname = s[:pos]
            s = s[pos:]
            print((explicit_modname, s))
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
        if not piece:
            return None
        pieces.append(piece)
        path = '.'.join(pieces[:-1])
        if path:
            path += "."
        else:
            path = None
        base = pieces[-1]
        groups = lambda: (explicit_modname, path, base, arg, retann)
#         if "[" not in s_orig:
#             try:
#                 assert old_m is not None, (s_orig, groups())
#                 old_groups = old_m.groups()
#                 assert old_groups == groups(), s_orig
#                 print(groups())
#                 print(old_groups)
#                 print("---")
#             except:
#                 import traceback
#                 traceback.print_stack()
#                 raise
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
