"""
Generates documentation for `pydrake`.
"""

from os.path import abspath, dirname

from serve_sphinx_base import main
from refresh_doc_modules import refresh_doc_modules

if __name__ == "__main__":
    input_dir = dirname(abspath(__file__))
    main(input_dir=input_dir, strict=False, src_func=refresh_doc_modules)
