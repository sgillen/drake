#!/usr/bin/env directorPython

# TODO: Make a simple camera viewing widget.

import sys
import argparse
import math
import time
import zlib
import threading

import numpy as np
import numpy.matlib

from vtk.util.numpy_support import vtk_to_numpy, get_vtk_array_type

from director import applogic
from director import consoleapp
from director import vtkAll as vtk
from director import lcmUtils
from director.timercallback import TimerCallback

import robotlocomotion as rl

import PythonQt
from PythonQt import QtGui

verbose = True

class ImageRetriever(object):
    def update_image(self, image):
        """
        This should update `image` in-place, either using `DeepCopy`, or by
        manually resizing, allocating, etc.
        If the image is updated, this should return True.
        Otherwise, it should return False.
        """
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
        # Must render first.
        self.view.render()

        # No idea why this is needed, but it works.
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


class ImageArrayWidget(object):
    def __init__(self, image_widgets):
        super(ImageArrayWidget, self).__init__()
        # Create widget and layouts
        self.widget = QtGui.QWidget()
        self.image_widgets = image_widgets
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

kDefaultChannel = "DRAKE_RGBD_CAMERA_IMAGES"
kDefaultFrameNames = ["color", "depth"]

class DrakeLcmImageViewer(object):
    def __init__(self, channel = kDefaultChannel, frame_names = None):
        self.channel = channel
        if frame_names is None:
            self._create_deferred()
        else:
            self._init_full(frame_names)

    def _init_full(self, frame_names):
        self.frame_names = frame_names
        self.subscriber = LcmImageArraySubscriber(
            self.channel, self.frame_names)

        retrievers = self.subscriber.create_retrievers()

        image_widgets = []
        for retriever in retrievers:
            image_widgets.append(ImageWidget(retriever))
        self.array_widget = ImageArrayWidget(image_widgets)

    def _create_deferred(self):
        # Defer creating viewer until we have a message.
        def callback(msg):
            print("Received on '{}'. Initializing frames...".format(
                self.channel))
            frame_names = [image.header.frame_name for image in msg.images]
            self._init_full(frame_names)
            lcmUtils.removeSubscriber(self._sub)
        print("Deferring creation until '{}' is received".format(self.channel))
        self._sub = lcmUtils.addSubscriber(
            self.channel, rl.image_array_t, callback)

def create_image(w, h, num_channels = 1, dtype=np.uint8):
    image = vtk.vtkImageData()
    image.SetWholeExtent(0, w - 1, 0, h - 1, 0, 0)
    image.SetExtent(image.GetWholeExtent())
    image.SetSpacing(1., 1., 1.)
    image.SetOrigin(0., 0., 0.)
    image.SetNumberOfScalarComponents(num_channels)
    image.SetScalarType(get_vtk_array_type(dtype))
    image.AllocateScalars()
    print("Create: np = {}".format((w, h, num_channels)))
    print("  vtk = {}".format(image.GetDimensions()))
    return image

def create_image_if_needed(w, h, num_channels, dtype, image_in):
    # If possible, use existing memory.
    if False: # image_in is not None:
        dim = (w, h, num_channels)
        attrib_out = (dim, dtype)
        data_in = vtk_image_to_numpy(image_in)
        attrib_in = (data_in.shape, data_in.dtype)
        if attrib_in == attrib_out:
            return (image_in, data_in)
    # Otherwise, create new image.
    image = create_image(w, h, num_channels, dtype)
    data = vtk_image_to_numpy(image)
    return (image, data)


def decode_image_t(msg, image_in = None):
    """
    Decode image_t to vtkImageData, using an existing image if it is compatible.
    """
    rli = rl.image_t
    print("\n\n")
    w = msg.width
    h = msg.height
    assert msg.compression_method == rli.COMPRESSION_METHOD_ZLIB
    pixel_desc = (msg.pixel_format, msg.channel_type)
    if pixel_desc == (rli.PIXEL_FORMAT_RGBA, rli.CHANNEL_TYPE_UINT8):
        print("rgba")
        num_channels = 4
        dtype = np.uint8
    elif pixel_desc == (rli.PIXEL_FORMAT_DEPTH, rli.CHANNEL_TYPE_FLOAT32):
        num_channels = 1
        dtype = np.float32
    elif pixel_desc == (rli.PIXEL_FORMAT_LABEL, rli.CHANNEL_TYPE_INT16):
        num_channels = 1
        dtype = np.int16
    else:
        raise Exception("Unsupported pixel type: {}".format(pixel_desc))
    bytes_per_pixel = np.dtype(dtype).itemsize * num_channels
    assert msg.row_stride == msg.width * bytes_per_pixel

    (image, data) = create_image_if_needed(w, h, num_channels, dtype, image_in)
    # TODO(eric.cousineau): Consider using `data`s buffer, if possible.
    # Can decompress() somehow use an existing buffer in Python?
    data_in = np.frombuffer(zlib.decompress(msg.data), dtype=dtype)
    data_in.shape = (w, h, num_channels)
    # Copy data over.
    data[:] = data_in[:]
    # Return image.
    return image

def vtk_image_to_numpy(image):
    data = vtk_to_numpy(image.GetPointData().GetScalars())
    print(data.shape, data.dtype)
    print(image.GetDimensions())
    data.shape = image.GetDimensions()
    return data

class LcmImageInfo(object):
    def __init__(self):
        self.image = None  # vtkImageData
        self.utime = 0
        self.lock = threading.Lock()

class LcmImageRetriever(ImageRetriever):
    def __init__(self, info):
        self.info = info
        self.prev_utime = 0

    def update_image(self, image_out):
        with self.info.lock:
            cur_utime = self.info.utime
            if self.prev_utime <= cur_utime:
                return False
            self.prev_utime = cur_utime
            image = self.info.image
            assert(image is not None)
            image_out.DeepCopy(image)


class LcmImageArraySubscriber(object):
    def __init__(self, channel = kDefaultChannel,
                 frame_names = kDefaultFrameNames):
        self.channel = channel
        self.frame_names = frame_names
        self.infos = {}
        for frame_name in self.frame_names:
            self.infos[frame_name] = LcmImageInfo()
        self.subscriber = lcmUtils.addSubscriber(
            channel, rl.image_array_t, self.on_message)

    def create_retrievers(self):
        retrievers = []
        for info in self.infos.itervalues():
            retrievers.append(LcmImageRetriever(info))
        return retrievers

    def on_message(self, msg):
        issues = []

        msg_frame_names = [image.header.frame_name for image in msg.images]
        for (frame_name, info) in self.infos.iteritems():
            if frame_name not in msg_frame_names:
                issues.append("Did not find '{}' in message".format(frame_name))
                continue
            index = msg_frame_names.index(frame_name)
            msg_image = msg.images[index]
            with info.lock:
                info.utime = image.header.utime
                info.image = decode_image_t(msg_image, info.image)

        if verbose:
            extra_frame_names = set(msg_frame_names) - set(self.frame_names)
            for frame_name in extra_frame_names:
                issues.append("Got extra image '{}'".format(frame_name))

        if issues:
            print("For image channel '{}' ({} images), with frames {}:".format(
                self.channel, msg.num_images, msg_frame_names))
            for issue in issues:
                print("  {}".format(issue))


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

        image_out.DeepCopy(self.image)
        return True

if __name__ == "__main__":
    has_app = 'app' in globals()

    parser = argparse.ArgumentParser()
    parser.add_argument('--test_image', action='store_true',
        help = "Use a test image retriever.")
    parser.add_argument('--channel', type=str, default=kDefaultChannel)
    parser.add_argument('--frame_names', type=str, nargs='+', default=None,
        help="By default, will populate with first set of frames. Otherwise, can be manually specified")
    args, _ = parser.parse_known_args()

    if args.test_image:
        image_viewer = ImageArrayWidget([
            ImageWidget(TestImageRetriever())
            ])
    else:
        image_viewer = DrakeLcmImageViewer(args.channel, args.frame_names)

    if not has_app:
        app = consoleapp.ConsoleApp()
        app.start()

