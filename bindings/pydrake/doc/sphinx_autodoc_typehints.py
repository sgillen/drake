# http://www.sphinx-doc.org/en/master/extdev/appapi.html#sphinx.application.Sphinx.add_autodocumenter
from __future__ import print_function

from sphinx.ext import autodoc

from pydrake.util.cpp_template import TemplateBase


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

    # def import_object(self):
    #     out = autodoc.ModuleLevelDocumenter.import_object(self)
    #     print("import_object: ", out)
    #     return out

    def check_module(self):
        return True
        # out = autodoc.ModuleLevelDocumenter.check_module(self)
        # print("check_module: ", out)
        # return out

    # def generate(self, **kwargs):



def setup(app):
    app.add_autodocumenter(TemplateDocumenter)
    return dict(parallel_read_safe=True)
