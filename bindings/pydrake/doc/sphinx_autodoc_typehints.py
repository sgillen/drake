# http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter
from __future__ import print_function

import re

from sphinx.ext import autodoc

from pydrake.util.cpp_template import TemplateBase


#: extended signature RE: with explicit module name separated by ::
autodoc.py_ext_sig_re = re.compile(
    r'''^ ([\w.]+::)?            # explicit module name
          ([\w.]+\.)?            # module and/or class name(s)
          ([\w\[\]]+)  \s*             # thing name
          (?: \((.*)\)           # optional: arguments
           (?:\s* -> \s* (.*))?  #           return annotation
          )? $                   # and nothing more
          ''', re.VERBOSE)


class TemplateDocumenter(autodoc.ModuleLevelDocumenter):
    """
    Specialized Documenter subclass for templates.
    """
    objtype = 'template'
    member_order =  autodoc.ClassDocumenter.member_order
    directivetype = 'attribute'
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
        assert name.endswith(']')
        for param in obj.param_list:
            inst = obj[param]
            if inst.__name__ == name:
                print(" - ", name, inst)
                return inst
        exit(10)
        raise RuntimeError("Not a template?")
    print(obj, name, *defargs)
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
