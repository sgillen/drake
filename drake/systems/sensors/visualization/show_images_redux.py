#!/usr/bin/env directorPython

# TODO: Make a simple camera viewing widget.

import argparse
import numpy as np
import numpy.matlib

from vtk.util.numpy_support import vtk_to_numpy, get_vtk_array_type

from director import applogic
from director import consoleapp
from director import vtkAll as vtk
from director.timercallback import TimerCallback

import PythonQt
from PythonQt import QtGui

import math
import time

class ImageRetriever(object):
    def update_image(self, image):
        raise Exception("Please implement")

# This is more like director...CameraImageView than ImageWidget
class ImageWidget(object):
    def __init__(self, image_retriever):
        self.name = 'Image View'
        self.view = PythonQt.dd.ddQVTKWidgetView()
        self.image_retriever = image_retriever

        self.image = vtk.vtkImageData()
        self.initialized = False
        self.prev_shape = (0, 0, 0)

        # Initialize the view.
        self.view.installImageInteractor()
        # Add actor.
        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInput(self.image)
        self.imageActor.SetVisibility(False)
        self.view.renderer().AddActor(self.imageActor)

        self.view.orientationMarkerWidget().Off()
        self.view.backgroundRenderer().SetBackground(0,0,0)
        self.view.backgroundRenderer().SetBackground2(0,0,0)

        # Add timer.
        self.render_timer = TimerCallback(
            targetFps = 60,
            callback = self.render)
        self.start_time = time.time()
        self.render_timer.start()

    def render(self):
        if not self.view.isVisible():
            return

        has_new = self.image_retriever.update_image(self.image)
        assert(isinstance(has_new, bool))
        if not has_new:
            return

        if not self.initialized:
            # Ensure it is visible.
            self.initialized = True
            self.imageActor.SetVisibility(True)

        cur_shape = self.image.GetDimensions()
        if self.prev_shape != cur_shape:
            # Fit image to view.
            self.fit_image_to_view()
            # Update.
            self.prev_shape = cur_shape

        self.view.render()

    def fit_image_to_view(self):
        print "Fit"
        # Must render first.
        self.view.render()

        camera = self.view.camera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0,0,0)
        camera.SetPosition(0,0,-1)
        camera.SetViewUp(0,-1, 0)
        self.view.resetCamera()

        imageWidth, imageHeight = self.image.GetDimensions()[:2]
        viewWidth, viewHeight = self.view.renderWindow().GetSize()

        aspect_ratio = float(viewWidth) / viewHeight
        parallel_scale = max(imageWidth / aspect_ratio, imageHeight) / 2.0
        camera.SetParallelScale(parallel_scale)

def create_image(w, h, num_channels = 1, dtype=np.uint8):
    num_components = 1
    image = vtk.vtkImageData()
    image.SetWholeExtent(0, w - 1, 0, h - 1, 0, 0)
    image.SetExtent(image.GetWholeExtent())
    image.SetSpacing(1., 1., 1.)
    image.SetOrigin(0., 0., 0.)
    image.SetNumberOfScalarComponents(num_channels)
    image.SetScalarType(get_vtk_array_type(dtype))
    image.AllocateScalars()
    return image

def vtk_image_to_numpy(image):
    data = vtk_to_numpy(image.GetPointData().GetScalars())
    data.shape = image.GetDimensions()
    return data

class TestImageRetriever(ImageRetriever):
    def __init__(self):
        self.start_time = time.time()
        self.image = create_image(640, 480)
        self.has_switched = False

    def update_image(self, image_out):
        t = time.time() - self.start_time

        if t > 2 and not self.has_switched:
            # Try changing the size.
            self.image = create_image(480, 640)
            self.has_switch = True

        data = vtk_image_to_numpy(self.image)
        w, h = data.shape[:2]

        x = np.matlib.repmat(
            np.linspace(0, 255., w).reshape(-1, 1),
            1, h)

        T = 1
        w = 2 * math.pi / T
        s = (math.sin(w * t) + 1) / 2.
        
        data[:, :, 0] = x * s
        print(data.shape)

        image_out.DeepCopy(self.image)
        return True

class DrakeImageViewer(object):
    def __init__(self):
        self.create_window()

    def create_window(self):
        print "Creating"
        self.image_widgets = [
            ImageWidget(TestImageRetriever())
        ]

        # Create widget and layouts
        self.widget = QtGui.QWidget()
        self.layout = QtGui.QHBoxLayout(self.widget)
        for image_widget in self.image_widgets:
            self.layout.addWidget(image_widget.view)
        self.layout.setContentsMargins(0, 0, 0, 0)

        default_width = 640
        default_height = 480
        dim = [
            default_width * len(self.image_widgets),
            default_height]

        self.widget.resize(*dim)
        self.widget.show()

if __name__ == "__main__":
    image_viewer = DrakeImageViewer()

    has_app = 'app' in globals()
    if not has_app:
        app = consoleapp.ConsoleApp()
        app.start()
