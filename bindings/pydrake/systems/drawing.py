"""
Provides general visualization utilities. This is NOT related to `rendering`.

Note:
    This is an optional module, dependent on `pydot` and `matplotlib` being
    installed.
"""

from tempfile import NamedTemporaryFile
from pydrake.common import temp_directory
import matplotlib.pyplot as plt
import pydot

from pydrake.systems.framework import LeafSystem_
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.util.cpp_template import TemplateFunction


# TODO(eric.cousineau): Move `plot_graphviz` to something more accessible to
# `call_python_client`.


def plot_graphviz(dot_text):
    """Renders a DOT graph in matplotlib."""
    # @ref https://stackoverflow.com/a/18522941/7829525
    # Tried (reason ignored): pydotplus (`pydot` works), networkx
    # (`read_dot` does not work robustly?), pygraphviz (coupled with
    # `networkx`).
    g = pydot.graph_from_dot_data(dot_text)
    if isinstance(g, list):
        # Per Ioannis's follow-up comment in the above link, in pydot >= 1.2.3
        # `graph_from_dot_data` returns a list of graphs.
        # Handle this case for now.
        assert len(g) == 1
        g = g[0]
    f = NamedTemporaryFile(suffix='.png', dir=temp_directory())
    g.write_png(f.name)
    plt.axis('off')

    return plt.imshow(plt.imread(f.name), aspect="equal")


def plot_system_graphviz(system):
    """Renders a System's Graphviz representation in `matplotlib`."""
    return plot_graphviz(system.GetGraphvizString())


class Test(object):
    """Test 1"""

    def __init__(self, y):
        """Ctor"""
        pass

    def method_1(self, x):
        """Test 1 method"""
        pass

    class Nested(object):
        """Nested 2"""
        def method_2(self, x):
            """Nested 2 method"""
            pass


@TemplateSystem.define("MySystem_")
def MySystem_(T):

    class Impl(LeafSystem_[T]):
        """
        Example class.
        """
        def _construct(self, value, converter=None):
            LeafSystem_[T].__init__(self, converter=converter)
            self.value = value

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, other.value, converter=converter)

        def my_method(self, x):
            return "Hello"

    return Impl

MySystem = MySystem_[None]  # Default instantiation.

@TemplateFunction.define("MyFunc", param_list=[(int,), (float,)])
def MyFunc(param):
    T, = param

    def impl(x, y):
        """
        Instantiation. yeah.
        """
        return (T, x, y)

    return impl
