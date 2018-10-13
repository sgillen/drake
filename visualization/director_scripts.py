"""Amalgam of visualizer scripts."""

from drake.visualization import (
    director_contact,
    director_frames,
    director_images,
    director_time,
)

assert __name__ == "__main__"
contact_viz = director_contact.init_visualizer()
frame_viz = director_frames.init_visualizer()
time_viz = director_time.init_visualizer()
image_viewer = director_images.init_visualizer(_argv)
