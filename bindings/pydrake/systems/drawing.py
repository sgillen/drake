# @file
# Provides general visualization utilities. This is NOT related to `rendering`.
# @note This is an optional module, dependent on `pydot` and `matplotlib` being
# installed.

from StringIO import StringIO

import pydot  # sudo pip install pydot
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def plot_dot(dot_text):
    """Renders a DOT graph in matplotlib."""
    # @ref https://stackoverflow.com/a/18522941/7829525
    # Tried (reason ignored): pydotplus (`pydot` works), networkx
    # (`read_dot` does not work robustly?), pygraphviz (coupled with
    # `networkx`).
    # TODO(eric.cousineau): Incorporate this into `call_python_client`.
    g = pydot.graph_from_dot_data(dot_text)
    s = StringIO()
    g.write_png(s)
    s.seek(0)
    plt.axis('off')
    plt.imshow(plt.imread(s), aspect="equal")


def plot_system_graphviz(system):
    dot_text = str(system.GetGraphvizString())
    plot_dot(dot_text)
