# Visualization

Contains the following `director` scripts:

*   `director_contact` - Contact visualization
*   `director_frame` - Frame visualization
*   `director_image` - Image visualization
*   `director_time` - Simulation time and real time factor display

These scripts are enabled in `//tools:drake_visualizer` by default.
They can be toggled by supplying `--drake_scripts=...` to `drake_visualizer`.

Examples:

```sh
bazel build //tools:drake_visualizer

./bazel-bin/tools/drake_visualizer  # All scripts
./bazel-bin/tools/drake_visualizer  