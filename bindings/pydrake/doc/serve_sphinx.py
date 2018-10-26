"""
Generates documentation for `pydrake`.
"""

from serve_sphinx_base import main
from refresh_doc_modules import refresh_doc_modules

if __name__ == "__main__":
    main(src_func=refresh_doc_modules)
