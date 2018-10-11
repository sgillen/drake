# Amalgam of visualizer scripts.

from drake.attic.multibody.rigid_body_plant.visualization import (
    contact_viz as contact_viz_module,
    show_frames,
    show_time,
)
from drake.systems.sensors.visualization import show_images


assert __name__ == "__main__"
contact_viz = contact_viz_module.init_visualizer()
frame_viz = show_frames.init_visualizer()
time_viz = show_time.init_visualizer()
image_viewer = show_images.init_visualizer(_argv)
