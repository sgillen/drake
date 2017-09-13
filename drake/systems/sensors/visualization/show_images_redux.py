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
        w = 20
        h = 20
        num_components = 1

        image = vtk.vtkImageData()
        image.SetWholeExtent(0, w - 1, 0, h - 1, 0, 0)
        image.SetExtent(image.GetWholeExtent())
        image.SetSpacing(1., 1., 1.)
        image.SetOrigin(0., 0., 0.)
        image.SetNumberOfScalarComponents(num_components)
        image.SetScalarType(vtk.VTK_UNSIGNED_SHORT)
        image.AllocateScalars()

        data = vtk_to_numpy(image.GetPointData().GetScalars())
        data.shape = image.GetDimensions()
        t = time.time() - self.start_time
        value = (255 * (t / 1.)) % 256
        data[:] = value

        self.image.DeepCopy(image)
        print("Updated: {}".format(value))
        print("  {}".format(image.GetPointData().GetArray(0).GetValue(10)))

    def render(self):
        if not self.view.isVisible():
            return
        self.update_image()
        print("Render")
        if not self.initialized:
            self.initialized = True
            self.imageActor.SetVisibility(True)
        self.view.render()

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

        print "Showing"

        # # Add shortcuts.
        # applogic.addShortcut(
        #     self.widget, 'Ctrl+Q', consoleapp.ConsoleApp.quit)
        # applogic.addShortcut(
        #     self.widget, 'F8', consoleapp.ConsoleApp.showPythonConsole)

if __name__ == "__main__":
    image_viewer = DrakeImageViewer()

    has_app = 'app' in globals()
    if not has_app:
        app = consoleapp.ConsoleApp()
        app.start()
