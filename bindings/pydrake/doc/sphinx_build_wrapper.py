# -*- coding: UTF-8 -*-

import os
import sys

sys.path.insert(0, '/home/eacousineau/proj/tri/repo/externals/sphinx')

runfiles_dir = os.environ.get('DRAKE_BAZEL_RUNFILES')
assert runfiles_dir, 'Environment variable DRAKE_BAZEL_RUNFILES is NOT set.'
sphinx_build_path = os.path.join(runfiles_dir, 'external/sphinx/sphinx-build')
assert os.path.exists(sphinx_build_path), \
    'Path {} does NOT exist.'.format(sphinx_build_path)
os.environ['LANG'] = 'en_US.UTF-8'
argv = [sphinx_build_path] + sys.argv[1:]

from sphinx import main
# import trace

# tracer = trace.Trace(ignoredirs=sys.path, trace=1, count=0) #['/usr']
# sys.exit(tracer.run('main(sys.argv)'))

sys.exit(main(sys.argv))

# os.execv(argv[0], argv)
