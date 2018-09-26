#!/usr/bin/python

# find . ../../../drake/bindings/pydrake -name *_py*.cc | xargs ./tmp_sub_doc.py -i

import re
from common_scripts.text_processor import TextProcessor

class Custom(TextProcessor):
    def __init__(self):
        TextProcessor.__init__(self, skipArgs = True, description = "Apply python regex to file")
        # self.parser.add_argument('pattern', type = str)
        # self.parser.add_argument('replace', type = str)
        self.addArguments()
        
    def process(self, oldText):
        define_pattern = r"#define D\(...\) DOC\((.*?)__VA_ARGS__\)"

        def define_sub(m):
            pieces = ["pydrake_doc"] + m.group(1).split(", ")
            pieces.remove('')
            return "auto& doc = " + ".".join(pieces) + ";"

        usage_pattern = r"\bD\((.*?)\)"

        def usage_sub(m):
            pieces = m.group(1).split(", ")
            suffix = ""
            try:
                index = int(pieces[-1])
                pieces = pieces[:-1]
                suffix = "_{}".format(index)
            except ValueError:
                pass
            if len(pieces) > 1 and pieces[-1] == pieces[-2]:
                pieces[-1] = "ctor"
            return "doc." + ".".join(pieces) + ".doc" + suffix

        new = oldText
        new = re.sub(define_pattern, define_sub, new, re.MULTILINE | re.DOTALL)
        new = re.sub(usage_pattern, usage_sub, new, re.MULTILINE | re.DOTALL)
        return new

Custom().main()
