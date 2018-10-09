# http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter
from __future__ import print_function

# New sphinx:
#  pip install typing
#  sudo apt install python-packaging

from collections import namedtuple
import re
from functools import wraps

import sphinx.domains.python as pydoc
from sphinx.ext import autodoc

from pydrake.util.cpp_template import TemplateBase, TemplateMethod

# import sys
# sys.stdout = sys.stderr

def rindex(s, sub):
    return len(s) - s[::-1].index(sub) - len(sub)

#: extended signature RE: with explicit module name separated by ::
class IrregularExpression(object):
    FakeMatch = namedtuple('FakeMatch', 'groups')
    py_sig_old = autodoc.py_ext_sig_re
    py_sig = re.compile(
        r'''^     (\w.*?) \s*               # symbol
                  (?:
                      \((.*)\)              # optional: arguments
                      (?:\s* -> \s* (.*))?  # return annotation
                  )? $
              ''', re.VERBOSE)

    def __init__(self, extended):
        self.extended = extended

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
        path = ''
        base = ''
        num_open = 0
        i = 0
        for c in s:
            if c.isspace() or c == '(':
                break
            if num_open == 0 and c == '.':
                path += base + "."
                base = ''
            else:
                if c == '[':
                    num_open += 1
                elif c == ']':
                    num_open -= 1
                base += c
            i += 1
        if not base:
            # Nothing worth keeping.
            return None
        if not path:
            # Clear out.
            path = None
        if self.extended:
            groups = lambda: (explicit_modname, path, base, arg, retann)
        else:
            assert explicit_modname is None
            groups = lambda: (path, base, arg, retann)
        return self.FakeMatch(groups)


class TemplateDocumenter(autodoc.ModuleLevelDocumenter):
    """
    Specialized Documenter subclass for templates.
    """
    #
    objtype = 'template'
    member_order =  autodoc.ClassDocumenter.member_order
    directivetype = 'template'
    option_spec = {}
    # Take preference over Attributes.
    priority = 1 + autodoc.AttributeDocumenter.priority

    @classmethod
    def can_document_member(cls, member, membername, isattr, parent):
        ret = isinstance(member, TemplateBase)
        print("check", type(member), membername)
        if ret:
            print("CAN", member)
        return ret

    def get_object_members(self, want_all):
        members = []
        is_method = isinstance(self.object, TemplateMethod)
        for param in self.object.param_list:
            instantiation = self.object[param]
            if is_method:
                instantiation = wraps(instantiation)(instantiation)
                instantiation.__name__ = self.object._instantiation_name(param)
                print("overwrite", instantiation)
            members.append((instantiation.__name__, instantiation))
        return False, members

    def import_object(self):
        out = autodoc.ModuleLevelDocumenter.import_object(self)
        print("import_object: ", self.object_name, out)
        self._DBG = True
        return out

    def check_module(self):
        # TODO(eric.cousineau): Filter out `TemplateBase` instances based on their originating module.
        # `autodoc` won't catch it normally because an instance does not have `__module__` normally defined.
        return True
        # out = autodoc.ModuleLevelDocumenter.check_module(self)
        # print("check_module: ", out)
        # return out

    def add_directive_header(self, sig):
        autodoc.ModuleLevelDocumenter.add_directive_header(self, sig)

        sourcename = self.get_sourcename()
        self.add_line(u'', sourcename)
        names = []
        for param in self.object.param_list:
            # TODO(eric.cousineau): Use attribute aliasing already present in autodoc.
            rst = ":class:`{}`".format(self.object[param].__name__)
            names.append(rst)
        self.add_line(u"   Instantiations: {}".format(", ".join(names)), sourcename)


def tpl_getter(obj, name, *defargs):
    if "[" in name:
        assert name.endswith(']'), name
        for param in obj.param_list:
            inst = obj[param]
            if inst.__name__ == name:
                return inst
        raise RuntimeError("Not a template?")
    return autodoc.safe_getattr(obj, name, *defargs)


old = autodoc.Documenter.filter_members

def rewrite(self, members, want_all):
    ret = old(self, members, want_all)
    s = []
    for (name, member, isattr) in ret:
        s.append([name, isattr])
        if name == "MyMethod":
            print(" - ", name, member)
    print(("rewrite", self.object_name, s))
    return ret

autodoc.Documenter.filter_members = rewrite


# class PyCppTemplate(pydoc.PyClasslike):
#     pass

pydoc.PythonDomain.directives['template'] = pydoc.PyClasslike


def setup(app):
    app.add_autodoc_attrgetter(TemplateBase, tpl_getter)
    app.add_autodocumenter(TemplateDocumenter)
    # Hack regular expressions to make them irregular (nested).
    autodoc.py_ext_sig_re = IrregularExpression(extended=True)
    pydoc.py_sig_re = IrregularExpression(extended=False)
    return dict(parallel_read_safe=True)


# # from pydrake.all import Adder_
# import pydrake.all
# Adder_ = autodoc.safe_getattr(pydrake.all, 'Adder_')
# print(Adder_)
# exit(10)
