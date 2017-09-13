#!/usr/bin/env directorPython

# TODO: Make a simple camera viewing widget.

import argparse
import numpy as np

from vtk.util.numpy_support import vtk_to_numpy

from director import applogic
from director import consoleapp
from director import vtkAll as vtk
from director.timercallback import TimerCallback

import PythonQt
from PythonQt import QtGui

import math
import time

# This is more like director...CameraImageView than ImageWidget
class ImageWidget(object):
    def __init__(self):
        self.name = 'Image View'
        self.view = PythonQt.dd.ddQVTKWidgetView() #applogic.getViewManager().createView(self.name, 'VTK View')

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

    def update_image(self):
        t = time.time() - self.start_time

        if t < 2:
            w = 640
            h = 480
        else:
            w = 3
            h = 6
        num_components = 1

        image = vtk.vtkImageData()
        image.SetWholeExtent(0, w - 1, 0, h - 1, 0, 0)
        image.SetExtent(image.GetWholeExtent())
        image.SetSpacing(1., 1., 1.)
        image.SetOrigin(0., 0., 0.)
        image.SetNumberOfScalarComponents(num_components)
        image.SetScalarType(vtk.VTK_UNSIGNED_CHAR)
        image.AllocateScalars()

        data = vtk_to_numpy(image.GetPointData().GetScalars())
        data.shape = image.GetDimensions()

        s = t % 1.
        data[:, :, 0] = 255. * s

        self.image.DeepCopy(image)

    def render(self):
        if not self.view.isVisible():
            return
        self.update_image()

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

class DrakeImageViewer(object):
    def __init__(self):
        self.create_window()

    def create_window(self):
        print "Creating"
        self.image_widgets = [ImageWidget()]

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
