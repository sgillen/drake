# Due to dependency cycle, only try to import attic symbols if they're
# available.
try:
    pass # from pydrake.attic.systems.sensors import *
except ImportError as e:
    pass
